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

#include "SCT.h"

// #include <map>

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

    /* List of states */
    enum class RobotState {
        LEADER = 0,
        FOLLOWER,
        CHAIN
    } currentState;

    /* Structure to store incoming data received from other robots */
    // struct Message {
    //     RobotState state;
    //     std::string id;
    //     UInt8 teamid;
    //     CVector2 direction;
    //     bool hasSeenChain; // FOLLOWER
    //     std::vector<std::string> connections; // CHAIN
    // };

public:

    /* Class constructor. */
    CLeader();

    /* Class destructor. */
    virtual ~CLeader() {}

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
    * Sets the list of waypoints to visit.
    */
    void SetWaypoints(const std::queue<CVector2> waypts);

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
    * Calculates the vector to the next waypoint.
    */
    virtual CVector2 VectorToWaypoint();

    /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
    void SetWheelSpeedsFromVector(const CVector2& c_heading);

    /*
    * Print robot id.
    */
    void PrintName();

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

    /* Flag to know whether this robot is selected */
    bool m_bSelected;

    /* The control vector */
    CVector2 m_cControl;

    /* Current team ID, which is the number of the leader ID (e.g. L1 -> 1) */
    UInt8 teamID;

    /* Messages received from nearby robots */
    // std::vector<Message> teamMsgs;
    // std::vector<Message> chainMsgs;
    // std::vector<Message> otherLeaderMsgs;
    // std::vector<Message> otherTeamMsgs;

    /* Outgoing message */
    CByteArray msg;
    size_t msg_index = 0;

    /* Ordered list of waypoints to visit */
    std::queue<CVector2> waypoints;

    /* Incoming message buffer (occurances of public uncontrollable events) */
    // std::map<size_t, bool> pub_events;

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */
    /* Consider it has arrived to a goal/waypoint if it is within a threshold */
    Real goalRange;
};

#endif
