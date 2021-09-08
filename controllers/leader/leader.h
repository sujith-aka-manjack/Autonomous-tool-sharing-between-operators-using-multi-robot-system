/*
 * AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 *
 * An example controller for running SCT with the e-puck.
 *
 * The controller uses the supervisors generated in Nadzoru to 
 * determine its next action in each timestep.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/leader_follower.argos
 * 
 * This example has been modified from the following examples provided in argos3-examples: https://github.com/ilpincy/argos3-examples/
 *   - argos3-examples/controllers/epuck_obstacleavoidance/
 *   - argos3-examples/controllers/footbot_manualcontrol/
 */

#ifndef LEADER_H
#define LEADER_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include "SCT_leader.h"
#include "SCT_leader_exchange.h"

#include <utility/pid.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CLeader : public CCI_Controller {

public:

    struct SWheelTurningParams {
        /*
        * The turning mechanism.
        * The robot can be in three different turning states.
        */
        enum ETurningMechanism
        {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        } TurningMechanism;
        /*
        * Angular thresholds to change turning state.
        */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;

        void Init(TConfigurationNode& t_tree);
    };

    /*
    * The following variables are used as parameters for
    * tracking waypoints. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><leader_controller><parameters><waypoint_tracking>
    * section.
    */
    struct SWaypointTrackingParams {
        /* Target angle to waypoint in radians */
        Real TargetAngle;
        /* Parameters to be used for PID */
        Real Kp;
        Real Ki;
        Real Kd;
        /* Consider it has arrived to a goal/waypoint if it is within a threshold */
        Real thresRange;
        
        void Init(TConfigurationNode& t_node);
    };

    /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><leader_controller><parameters><team_flocking>
    * section.
    */
    struct SFlockingInteractionParams {
        /* Target robot-robot distance in cm */
        Real TargetDistance;
        /* Gain of the Lennard-Jones potential */
        Real Gain;
        /* Exponent of the Lennard-Jones potential */
        Real Exponent;

        void Init(TConfigurationNode& t_node);
        Real GeneralizedLennardJones(Real f_distance);
        Real GeneralizedLennardJonesRepulsion(Real f_distance);
    };

    /* List of states */
    enum class RobotState {
        LEADER = 0,
        FOLLOWER,
        CONNECTOR,
        TRAVELER
    } currentState;

    /* Structure to store the connection to the leader/team */
    struct HopMsg {
        UInt8 count;
        std::string ID; // Robot with lower hop value (only used by connectors)
    };

    /*
    * Structure to store request/approval messages for extending the chain.
    * 
    *   - R (Request) : Follower sends to leader or connector 
    *   - A (Accept)  : Leader or connector sends to follower
    * 
    *       Structure: Type [1], sender ID [2], recipient ID [2], recipient team ID [1] (for Connector -> Follower accepts) 
    */
    struct ConnectionMsg {
        char type = 'N'; // R or A or N (none)
        std::string from;
        std::string to;
        UInt8 toTeam;
    };

    /* Message sent by a leader to other leaders */
    struct RelayMsg {
        char type = 'H'; // H (heart-beat) or R (request-robot) or A (acknowledge)
        std::string from;
        UInt16 time;
        std::string firstFollower; // First follower that received this message from a non-team robot
        UInt8 robot_num = 0;
    };

/* 
    * Structure to store incoming data received from other robots 
    * 
    * The raw messages are assumed to arrive in the following data structure:
    * 
    * |  (1)   |  (2)   |   (3)   |  (4)   |  (5)-(7)  | (8)-(16)  |  (17)-(29) | (30)-(34) | (35)-(37) | (38)-(54) |      (55)-(114)       | (115) |
    * -----------------------------------------------------------------------------------------------------------------------------------------------
    * | Sender | Sender | Sender  | Leader |   Team    | Hop count | Connection |  Shared   |   Teams   |   Relay   |      Connections      |  End  |
    * | State  |   ID   | Team ID | Signal |  Switch   |           |  Message   |  Message  |   Nearby  |  Message  | (2 bytes for ID x 30) | (255) |
    * 
    * 
    * - (4) Leader Signal
    *   - Leader    : task signal [1]
    * 
    * - (5)-(7) Team Switch Signal
    *   - Leader informs a follower to join another team
    *       - robotID [2]
    *       - teamID [1]
    * 
    * - (8)-(16) Hop count
    *   Prefix with number of messages (max 2) [1]
    *   - HopMsg (teamID [1], count [1], ID [2])
    * 
    * - (17)-(29) Connection Message
    *   Prefix with number of messages (max 2) [1]
    *   - ConnectionMsg [6]
    * 
    *       - Exchanging ConnectionMsg within a team:
    *           - If message destination is to leader, relay upstream
    *           - If message sender is the leader, relay downstream
    *
    *       - Exchanging ConnectionMsg between follower and connector:
    *           - Follower will send up to one request message (R)
    *           - Connector will send up to two approval messages (A)
    * 
    * - (30)-(34) Shared Message
    * 
    *       - Share information about the closest connector to the team
    *           - shareToLeader: Upstream (Follower to Leader)
    *           - shareToTeam  : Downstream (Leader to Follower)
    *       - Share information about the shortest distance to the other team (only when no connector is detected)
    *           - shareDist    : Upstream (Follower to Leader) 
    * 
    * - (35)-(37) Teams Nearby
    *   Prefix with number of teams nearby (max 2) [1]
    *   - teamID [1]
    * 
    *       - Used by connectors to determine whether other connectors can switch to a follower
    * 
    * - (38)-(54) Relay Message
    *   Prefix with number of messages (max 2) [1]
    *   - RelayMsg (Leader ID [2], Type [1], time sent [2], first follower [2], robot_num [1])
    * 
    *       - Message sent by a leader to other leaders
    * 
    */
    struct Message {
        
        /* Core */
        CVector2 direction;
        RobotState state;
        std::string ID;
        UInt8 teamID;

        /* Leader Signal */
        UInt8 leaderSignal;

        /* Team Switch */
        std::string robotToSwitch = "";
        UInt8 teamToJoin;

        /* Hop Count */
        std::map<UInt8, HopMsg> hops; // Key is teamID

        /* Connection Message*/
        std::vector<ConnectionMsg> cmsg;

        /* Shared Message */
        std::string shareToLeader = "";
        std::string shareToTeam = "";
        UInt8 shareDist = 255;

        /* Teams Nearby */
        std::vector<UInt8> nearbyTeams;

        /* Relay Message */
        std::vector<RelayMsg> rmsg;

        /* Detected neighbors */
        std::vector<std::string> connections;

        bool Empty();
    };

public:

    /* Class constructor. */
    CLeader();

    /* Class destructor. */
    virtual ~CLeader();

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_obstacleavoidance_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
    virtual void Reset();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    /*
    * Sets the selected flag on this robot.
    * When selected, a robot follows the control vector.
    */
    void Select();

    /*
    * Unsets the selected flag on this robot.
    * When unselected, a robot stays still.
    */
    void Deselect();

    /*
    * Sets the control vector.
    */
    void SetControlVector(const CVector2& c_control);

    /*
    * Sets the start/stop signal.
    */
    void SetSignal(const bool b_signal);

    /*
    * Get the next waypoint in the queue.
    */
    CVector2 GetNextWaypoint();

    /*
    * Sets the list of waypoints to visit.
    */
    void SetWaypoints(const std::queue<CVector2> waypts);

    /* 
    * Sets the current task info.
    */
    void SetTaskDemand(const UInt32 un_demand);

    /* 
    * Sets the current task info.
    */
    void SetMinimumCount(const UInt32 un_min);

    /*
    * Sets the current number of followers in the team.
    */
    void SetFollowerCount(const UInt32 un_count);

    /*
    * Get team ID.
    */
    virtual UInt8 GetTeamID();

    /*
    * Returns the timestep that this leader has sent a message to the other leader.
    * When it has not yet sent a time, it returns -1.
    */
    virtual Real GetLatestTimeSent();

    /*
    * Checks whether the leader has received a message from the other leader.
    * Returns the timestep that the message was sent, if message was received.
    */
    virtual Real GetLatestTimeReceived();

    /*
    * Get the total number of messages received from the other leader.
    */
    virtual Real GetTotalReceived();

    /*
    * Returns the last action made by the leader.
    */
    virtual std::string GetLastAction();

protected:

    /* 
    * Receive messages from neighboring robots.
    */
    virtual void GetMessages();

    /* 
    * Update sensor readings.
    */
    virtual void Update();

    /*
    * Returns whether it is near any other robot.
    */
    virtual bool IsNearRobot();

    /*
    * Upon receiving a Request message from follower, decide to send an Accept message (max 1).
    */
    virtual void ReplyToRequest();

    /*
    * Find the closest connector info that needs to be shared within the team.
    */
    virtual void SetConnectorToRelay();

    /*
    * Check whether a heart beat message is received from the other leader.
    */
    virtual void CheckHeartBeat();

    /*
    * Calculates the vector to the next waypoint.
    */
    virtual CVector2 VectorToWaypoint();

    /* 
    * Get a flocking vector between itself and the other robots not in the same team.
    */
    virtual CVector2 GetRobotRepulsionVector();

    /*
    * Get a repulsion vector from obstacles.
    */
    virtual CVector2 GetObstacleRepulsionVector();

    /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    * Used in manual control.
    */
    void SetWheelSpeedsFromVector(const CVector2& c_heading);

    /*
    * Gets a direction vector as input and transforms it into wheel actuation using a PID controller.
    * Used in autonomous control.
    */
    void SetWheelSpeedsFromVectorHoming(const CVector2& c_heading);

    /*
    * Print robot id.
    */
    void PrintName();

    /* Callback functions */
    virtual void Callback_Start(void* data);
    virtual void Callback_Stop(void* data);
    virtual void Callback_Message(void* data);
    virtual void Callback_Respond(void* data);
    virtual void Callback_Exchange(void* data);

    virtual unsigned char Check__Message(void* data);
    virtual unsigned char Check__Relay(void* data);
    virtual unsigned char Check__RequestL(void* data);
    virtual unsigned char Check_InputStart(void* data);
    virtual unsigned char Check_InputStop(void* data);
    virtual unsigned char Check_InputMessage(void* data);
    virtual unsigned char Check_InputExchange(void* data);

private:

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the e-puck proximity sensor */
    CCI_ProximitySensor* m_pcProximity;
    /* Pointer to the range-and-bearing actuator */
    CCI_RangeAndBearingActuator* m_pcRABAct;
    /* Pointer to the range-and-bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABSens;
    /* Pointer to the LEDs actuator */
    CCI_LEDsActuator* m_pcLEDs;
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor* m_pcPosSens;

    /* The turning parameters */
    SWheelTurningParams m_sWheelTurningParams;
    /* The waypoint tracking parameters */
    SWaypointTrackingParams m_sWaypointTrackingParams;
    /* The flocking interaction parameters between teammates. */
    SFlockingInteractionParams m_sTeamFlockingParams;

    /* Controller */
    leader::SCTPub* sct;
    bool exchangeUsed = false;
    // leader_exchange::SCTPub* sct;
    // bool exchangeUsed = true;

    /* Type of SCT being used */

    /* Flag to know whether this robot is selected */
    bool m_bSelected;

    /* The control vector */
    CVector2 m_cControl;

    /* The task signal */
    bool m_bSignal;

    /* Current team ID, which is the number of the leader ID (e.g. L1 -> 1) */
    UInt8 teamID;

    /* Proximity sensor readings */
    std::vector<Real> proxReadings;

    /* Messages received from nearby robots */
    std::vector<Message> teamMsgs;
    std::vector<Message> connectorMsgs;
    std::vector<Message> otherLeaderMsgs;
    std::vector<Message> otherTeamMsgs;
    std::vector<Message> travelerMsgs;

    /* Outgoing message */
    CByteArray msg;
    size_t msg_index = 0;

    /* PID to control the heading angle */
    PID* PIDHeading;

    /* Ordered list of waypoints to visit */
    std::queue<CVector2> waypoints;

    /* Task demand of the current working task */
    UInt32 currentTaskDemand;
    UInt32 previousTaskDemand; // Task demand from 10 timesteps ago

    int robotsNeeded; // Minimum number of robots needed to perform the current task

    /* Current number of followers in the team */
    int currentFollowerCount;

    /* Flag to know whether there is a neighbor */
    bool nearRobot;

    /* Info of the closest non team member to broadcast */
    RobotState closeState;
    Real closeDist;
    std::string closeID;

    /* Incoming message buffer (occurances of public uncontrollable events) */
    // std::map<size_t, bool> pub_events;

    /* Connection related info to send in the current timestep */
    std::vector<ConnectionMsg> cmsgToSend; 
    std::vector<std::pair<size_t,ConnectionMsg>> cmsgToResend; // ConnectionMsg attached with a timer. Messages gets added into cmsgToSend while timer is running
    std::string shareToTeam;

    std::vector<RelayMsg> rmsgToSend;
    std::vector<std::pair<size_t, RelayMsg>> rmsgToResend;
    Real lastSent;
    Real lastBeatTime;
    size_t beatReceived;
    std::string switchCandidate; // Robot that the leader could choose to switch to the other team
    std::string previousCandidate;
    bool notDecremented;

    /* Timer to count the timesteps for the initial communication to occur at the beginning of the simulation */
    size_t initStepTimer;

    /* Team switch variables */
    int numRobotsToSend;
    int numPreviousRequest;
    std::string robotToSwitch;
    UInt8 teamToJoin;

    /* Flag to indicate trigerring of uncontrollable events */
    bool receivedMessage, receivedRelay, receivedRequest, inputStart, inputStop, inputMessage;

    std::string acceptID;

    std::string lastAction;

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */
    /* The leader can move as long as it is withthin the minimum distance threshold from the robots */
    Real minDistanceFromRobot;

    /* Chain formation threshold */
    Real separationThres;

    size_t sendDuration;
};

#endif
