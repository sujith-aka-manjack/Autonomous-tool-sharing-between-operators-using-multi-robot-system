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
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><flocking>
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
    * Get a flocking vector between itself and the leader.
    */
    virtual CVector2 GetLeaderFlockingVector();

    /* 
    * Get a flocking vector between itself and the other followers in the same team.
    */
    virtual CVector2 GetTeamFlockingVector();

    /* 
    * Get a flocking vector between itself and the other robots not in the same team.
    */
    virtual CVector2 GetOtherRepulsionVector();

    /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
    virtual void SetWheelSpeedsFromVector(const CVector2& c_heading);

    /*
    * Find the distance between the leader and the closest chain robot or other leader detected.
    * If distance between the leader and the chain or other leader exceeds chainThreshold, 
    * become a chain robot.
    */
    virtual void CheckJoinChain();

    /*
    * Print robot id.
    */
    virtual void PrintName();

    /* Callback functions */
    virtual void Callback_Flock(void* data);
    virtual void Callback_Stop(void* data);
    virtual void Callback_JoinLeader(void* data);
    virtual void Callback_JoinChain(void* data);
    virtual void Callback_Wait(void* data);

    virtual unsigned char Check_LeaderNear(void* data);
    virtual unsigned char Check_LeaderFar(void* data);
    virtual unsigned char Check_SingleChain(void* data);
    virtual unsigned char Check_MultiChain(void* data);

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
    /* The flocking interaction parameters between teammates. */
    SFlockingInteractionParams m_sTeamFlockingParams;
    /* The flocking interaction parameters between leader. */
    SFlockingInteractionParams m_sLeaderFlockingParams;

    /* Controller */
    SCTProb* sct;

    /* Robot state */
    enum RobotState {
        LEADER = 0,
        FOLLOWER,
        CHAIN
    };

    /* Current team ID, which is the number of the leader ID (e.g. L1 -> 1) */
    size_t teamID;

    /* Vector to leader */
    CVector2 leaderVec;
    /* Vector to teammate */
    std::vector<CVector2> teamVecs;
    /* Vector to other robots */
    std::vector<CVector2> otherVecs;
    /* Vector to chain members */
    std::vector<CVector2> chainVecs;

    /* Outgoing message */
    CByteArray msg;
    size_t msg_index = 0;

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */
    /* Chain formation threshold */
    Real chainThreshold;

};

#endif
