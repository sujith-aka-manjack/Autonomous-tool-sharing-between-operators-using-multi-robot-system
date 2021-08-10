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
 *   - argos3-examples/controllers/footbot_flocking/
 */

#ifndef FOLLOWER_H
#define FOLLOWER_H

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

#include "SCT.h"
#include <utility/pid.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFollower : public CCI_Controller {

public:

    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><wheel_turning>
    * section.
    */
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
    * leader following. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><follower_controller><parameters><leader_flocking>
    * section.
    */
    struct SLeaderInteractionParams {
        /* Target leader-robot distance in cm */
        Real TargetDistance;
        /* Parameters to be used for PID */
        Real Kp;
        Real Ki;
        Real Kd;

        void Init(TConfigurationNode& t_node);
    };

    /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><follower_controller><parameters><team_flocking>
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
        CONNECTOR
    } currentState;

    /* List of move types available to the robot */
    enum class MoveType {
        STOP = 0,
        FLOCK
    } currentMoveType;

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

    /* 
    * Structure to store incoming data received from other robots 
    * 
    * The raw messages are assumed to arrive in the following data structure:
    * 
    *    |  (1)   |  (2)   |   (3)   |  (4)   | (5)-(13)  |  (14)-(26) | (27)-(30) |       (31)-(90)       | (91)  |
    *    -----------------------------------------------------------------------------------------------------------
    *    | Sender | Sender | Sender  | Leader | Hop count | Connection |  Shared   |      Connections      |  End  |
    *    | State  |   ID   | Team ID | Signal |           |  Message   |  Message  | (2 bytes for ID x 30) | (255) |
    * 
    * 
    * - (4) Leader Signal
    *   - Leader    : task signal [1]
    * 
    * - (5)-(13) Hop count
    *   Prefix with number of messages (max 2) [1]
    *   - HopMsg (teamID [1], count [1], ID [2])
    * 
    * - (14)-(26) Connection Message
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
    * - (27)-(30) Shared Message
    * 
    *       - Share information about the closest connector to the team
    *           - shareToLeader: Upstream (Follower to Leader)
    *           - shareToTeam  : Downstream (Leader to Follower)
    */
    struct Message {
        
        /* Core */
        CVector2 direction;
        RobotState state;
        std::string ID;
        UInt8 teamID;

        /* Leader Signal */
        UInt8 leaderSignal;

        /* Hop Count */
        std::map<UInt8, HopMsg> hops; // Key is teamID

        /* Connection Message*/
        std::vector<ConnectionMsg> cmsg;

        /* Update Message */
        std::string shareToLeader = "";
        std::string shareToTeam = "";

        /* Detected neighbors */
        std::vector<std::string> connections;

        bool Empty();
    };

public:

    /* Class constructor. */
    CFollower();

    /* Class destructor. */
    virtual ~CFollower();

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
    * Get team ID.
    */
    virtual UInt8 GetTeamID();

    /*
    * Set team ID.
    */
    virtual void SetTeamID(const UInt8 id);

    /*
    * Get hops count to teams.
    */
    virtual const std::map<UInt8, HopMsg>& GetHops() const;

    /*
    * Return whether the robot is working on a task.
    */
    virtual bool IsWorking();

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
    * Extract information about the leader from the team messages received.
    */
    virtual void GetLeaderInfo();

    /*
    * Return the closest non-team robot within its range.
    *
    * Connectors have higher priority than followers from another team.
    * @return Message received from the closest robot.
    */
    virtual Message GetClosestNonTeam();

    /*
    * Check whether it is the closest to a robot among its neighboring
    * team members that have observed that robot.
    * 
    * @param msg Message of the robot to check the distance with other team members.
    * @return boolean
    */
    virtual bool IsClosestToRobot(const Message& msg);

    /*
    * Check whether it has received an accept message.
    */
    virtual void CheckAccept();

    /*
    * Relay Request and Accept messages.
    *
    * Messages are relayed both upstream (to leader) and downstream (to the team).
    */
    virtual void SetCMsgsToRelay();

    /*
    * Find the closest connector info that needs to be shared within the team.
    */
    virtual void SetConnectorToRelay();

    /*
    * Update the hop count when forming part of the chain network.
    */
    virtual void UpdateHopCounts();

    /*
    * Check whether it has received any request messages and decide which to accept.
    *
    * Check all requests sent to itself and choose one to respond for each team.
    */
    virtual void CheckRequests();

    /*
    * Move wheels according to flocking vector
    */
    virtual void Flock();

    /* 
    * Get a flocking vector between itself and team members with the smallest hop count.
    */
    virtual CVector2 GetTeamFlockingVector();

    /* 
    * Get a repulsion vector between itself and all other robots.
    */
    virtual CVector2 GetRobotRepulsionVector();

    /*
    * Get a repulsion vector from obstacles.
    */
    virtual CVector2 GetObstacleRepulsionVector();

    /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
    virtual void SetWheelSpeedsFromVector(const CVector2& c_heading);

    /*
    * Print robot id.
    */
    virtual void PrintName();

    /* Callback functions */
    virtual void Callback_MoveFlock(void* data);
    virtual void Callback_MoveStop(void* data);
    virtual void Callback_TaskBegin(void* data);
    virtual void Callback_TaskStop(void* data);
    virtual void Callback_SetF(void* data);
    virtual void Callback_SetC(void* data);
    virtual void Callback_SendRL(void* data);
    virtual void Callback_SendRC(void* data);
    virtual void Callback_SendA(void* data);

    virtual unsigned char Check_CondC1(void* data);
    virtual unsigned char Check_NotCondC1(void* data);
    virtual unsigned char Check_CondC2(void* data);
    virtual unsigned char Check_NotCondC2(void* data);
    virtual unsigned char Check_CondC3(void* data);
    virtual unsigned char Check_NotCondC3(void* data);   
    virtual unsigned char Check_NearC(void* data);
    virtual unsigned char Check_NotNearC(void* data);
    virtual unsigned char Check_CondF1(void* data);
    virtual unsigned char Check_NotCondF1(void* data);
    virtual unsigned char Check_CondF2(void* data);
    virtual unsigned char Check_NotCondF2(void* data);
    virtual unsigned char Check_ReceiveR(void* data);
    virtual unsigned char Check_ReceiveA(void* data);
    virtual unsigned char Check_ReceiveNA(void* data);
    virtual unsigned char Check_ReceiveTB(void* data);
    virtual unsigned char Check_ReceiveTS(void* data);
    virtual unsigned char Check_TaskEnded(void* data);

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

    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;
    /* The flocking interaction parameters between leader. */
    SLeaderInteractionParams m_sLeaderFlockingParams;
    /* The flocking interaction parameters between teammates. */
    SFlockingInteractionParams m_sTeamFlockingParams;
    
    /* Weights for the flocking behavior */
    Real teamWeight;
    Real robotWeight;
    Real obstacleWeight;

    /* Controller */
    SCT* sct;

    /* Current team ID, which is the number of the leader ID (e.g. L1 -> 1) */
    UInt8 teamID;

    /* PID controller to calculate the force towards the leader */
    PID* pid;

    /* Outgoing message */
    CByteArray msg;
    size_t msg_index = 0;

    /* Messages received from nearby robots */
    Message leaderMsg;
    std::vector<Message> teamMsgs;
    std::vector<Message> connectorMsgs;
    std::vector<Message> otherLeaderMsgs;
    std::vector<Message> otherTeamMsgs;

    /* The number of hops from the leader to itself */
    UInt8 hopCountToLeader;  // default to 255 if unknown // (used in the FOLLOWER state)

    /* The number of hops to each team (used in the CONNECTOR state) */
    std::map<UInt8, HopMsg> hopsDict;
    std::map<UInt8, HopMsg> hopsCopy;  // The hop count info of the connector this robot will connect with (used in the FOLLOWER state)

    /* Sensor reading results */
    Message connectionCandidate; // (used in the FOLLOWER state)
    Message firstConnector; // The connector that a follower in the team should connect to next

    bool condC2;
    bool receiveR, receiveA, receiveNA;

    ConnectionMsg currentRequest, currentAccept; // (used in the FOLLOWER state)
    size_t requestTimer; // Remaining timesteps to wait since a request was made (used in the FOLLOWER state)
    UInt8 leaderSignal; // 0 = stop working on task, 1 = start working on task (used in the FOLLOWER state)

    std::unordered_map<UInt8, std::string> robotsToAccept; // List of robots to accept as connectors (used in the CONNECTOR state)

    /* Connection related info to send in the current timestep */
    std::vector<ConnectionMsg> cmsgToSend;
    std::vector<std::pair<size_t,ConnectionMsg>> cmsgToResend; // ConnectionMsg attached with a timer. Messages gets added into cmsgToSend while timer is running
    std::string shareToLeader, shareToTeam;

    /* Flag to indicate whether this robot is working on a task */
    bool performingTask;

    bool setCTriggered; // Flag to trigger the uncontrollable event taskEnded

    /* Timer to count the timesteps for the initial communication to occur at the beginning of the simulation */
    size_t initStepTimer;

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */
    /* Chain formation threshold */
    Real separationThres;
    Real joiningThres;

    size_t sendDuration;
    size_t waitRequestDuration;
};

#endif
