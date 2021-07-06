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

    /* 
    * Structure to store incoming data received from other robots 
    * 
    * The raw messages are assumed to arrive in the following data structure:
    * 
    *    |  (1)   |  (2)   |   (3)   |       (4)        |    (5)    |    (6)-(7)    |        (8)-(67)       |  
    *    ----------------------------------------------------------------------------------------------------
    *    | Sender | Sender | Sender  |      Signal      | Hop-count |   Connector   |      Connections      |
    *    | State  |   ID   | Team ID | (Only by leader) | to leader | Approval (ID) | (2 bytes for ID x 30) |
    * 
    */
    struct Message {
        CVector2 direction;
        RobotState state;
        std::string id;
        UInt8 teamid;
        UInt8 taskSignal;
        UInt8 hopCount;
        std::vector<std::string> connections;
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
    virtual void UpdateSensors();

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
    * Move wheels according to flocking vector
    */
    virtual void Flock();

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
    virtual void Callback_SetFS(void* data);
    virtual void Callback_SetCS(void* data);

    virtual unsigned char Check_ReceiveTB(void* data);
    virtual unsigned char Check_ReceiveTS(void* data);
    virtual unsigned char Check_DistFar(void* data);
    virtual unsigned char Check_DistNear(void* data);
    virtual unsigned char Check_IsNearest(void* data);
    virtual unsigned char Check_NotNearest(void* data);

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
    UInt8 hopCountToLeader;  // default to 255 if unknown

    /* Sensor reading results */
    Real minNonTeamDistance; // Distance to the closest non-team member
    bool isClosestToNonTeam;

    /* Flag to indicate whether this robot is working on a task */
    bool performingTask;

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */
    /* Chain formation threshold */
    Real separationThres;
    Real joiningThres;

};

#endif
