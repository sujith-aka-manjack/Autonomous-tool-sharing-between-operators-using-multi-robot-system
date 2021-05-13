/* Include the controller definition */
#include "leader.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

#include <algorithm>

/****************************************/
/****************************************/

void CLeader::SWheelTurningParams::Init(TConfigurationNode& t_node) {
    try {
        TurningMechanism = NO_TURN;
        CDegrees cAngle;
        GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
        HardTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
        SoftTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
        NoTurnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }
}

/****************************************/
/****************************************/

void CLeader::SWaypointTrackingParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_angle", TargetAngle);
      GetNodeAttribute(t_node, "kp", Kp);
      GetNodeAttribute(t_node, "ki", Ki);
      GetNodeAttribute(t_node, "kd", Kd);
      GetNodeAttribute(t_node, "thres_range", thresRange);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller waypoint tracking parameters.", ex);
   }
}

/****************************************/
/****************************************/
void CLeader::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CLeader::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CLeader::CLeader() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_pcLEDs(NULL),
    m_pcPosSens(NULL),
    m_bSelected(false),
    PIDHeading(NULL),
    closeToRobot(false) {}

/****************************************/
/****************************************/

void CLeader::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"            );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"          );

    /*
    * Parse the config file
    */
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
        /* Waypoint tracking */
        m_sWaypointTrackingParams.Init(GetNode(t_node, "waypoint_tracking"));
        /* Flocking-related */
        m_sTeamFlockingParams.Init(GetNode(t_node, "team_flocking"));
        /* Minimum distance from robot */
        GetNodeAttribute(GetNode(t_node, "team_distance"), "min_leader_robot_distance", minDistanceFromRobot);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Get team ID from leader ID */
    teamID = stoi(GetId().substr(1));

    /* Set state to leader */
    currentState = RobotState::LEADER;

    /* Set LED color */
    m_pcLEDs->SetAllColors(CColor::BLUE);

    /* Init PID Controller */
    PIDHeading = new PID(0.1,                             // dt  (loop interval time)
                         m_sWheelTurningParams.MaxSpeed,  // max
                         -m_sWheelTurningParams.MaxSpeed, // min
                         m_sWaypointTrackingParams.Kp,    // Kp
                         m_sWaypointTrackingParams.Ki,    // Ki
                         m_sWaypointTrackingParams.Kd);   // Kd

    Reset();
}

/****************************************/
/****************************************/

void CLeader::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    msg = CByteArray(16, 255);
    m_pcRABAct->SetData(msg);
    msg_index = 0;

    /* Reset the incoming public events */
    // pub_events.clear();
    // pub_events[EV_b] = false;
}

/****************************************/
/****************************************/

void CLeader::ControlStep() {

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/

    /* Clear messages received */
    teamMsgs.clear();
    chainMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();

    closeToRobot = false;

    // for(int i = 0; i < waypoints.size(); i++) {
    //     std::cout << waypoints[i].GetX() << "," << waypoints[i].GetY() << std::endl;
    // }

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();

    /*------------------------*/
    /* Update sensor readings */
    /*------------------------*/
    UpdateSensors();

    /*---------*/
    /* Control */
    /*---------*/

    /* Follow the control vector only if selected */
    if(m_bSelected) {
        SetWheelSpeedsFromVector(m_cControl);
    }
    else {
        if( !closeToRobot ) {
            /* Stop if other robots are too far from itself */
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        }
        else if( !waypoints.empty() ) {
            /* Calculate overall force applied to the robot */
            CVector2 waypointForce = VectorToWaypoint();        // Attraction to waypoint
            CVector2 otherForce = GetOtherRepulsionVector();    // Repulsion from other robots (other team and chain robots)
            CVector2 sumForce = waypointForce + otherForce;
            // std::cout << "waypointForce: " << waypointForce << std::endl;
            // std::cout << "otherForce: " << otherForce << std::endl;
            // std::cout << "sumForce: " << sumForce << std::endl;
            SetWheelSpeedsFromVectorHoming(GetOtherRepulsionVector() + VectorToWaypoint());
        }
        else
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    }

    /* Create new message */
    msg = CByteArray(16, 255);
    msg_index = 0;
    /* Set its state in msg */
    msg[msg_index++] = static_cast<UInt8>(currentState);
    /* Set sender ID in msg */
    msg[msg_index++] = teamID;  // For leader, ID = teamID
    /* Set team ID in msg */
    msg[msg_index++] = teamID;
    /* Set whether it has seen a chain */
    if( !chainMsgs.empty() || !otherLeaderMsgs.empty() || !otherTeamMsgs.empty() )
        msg[msg_index++] = 1;
    else
        msg[msg_index++] = 0;
    msg[msg_index++] = 1;

    /*--------------*/
    /* Send message */
    /*--------------*/
    m_pcRABAct->SetData(msg);

}

/****************************************/
/****************************************/

void CLeader::Select() {
   m_bSelected = true;
//    m_pcLEDs->SetAllColors(CColor::RED);
}

/****************************************/
/****************************************/

void CLeader::Deselect() {
   m_bSelected = false;
//    m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

void CLeader::SetControlVector(const CVector2& c_control) {
   m_cControl = c_control;
}

/****************************************/
/****************************************/

void CLeader::SetWaypoints(const std::queue<CVector2> waypts) {
    waypoints = waypts;
}

/****************************************/
/****************************************/

// void CLeader::GetMessages() {

//     /* Reset all public event occurances */
//     for(auto itr = pub_events.begin(); itr != pub_events.end(); ++itr) {
//         itr->second = false;
//     }

//     /* Get RAB messages from nearby e-pucks */
//     const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

//     if(! tMsgs.empty()) {
//         for(size_t i = 0; i < tMsgs.size(); ++i) {
//             size_t j = 0;
//             while(tMsgs[i].Data[j] != 255) {    // Check all events in the message
//                 unsigned char event = tMsgs[i].Data[j];
//                 pub_events[event] = true;   // If a public event has occured, set it to true
//                 j++;
//             }
//         }
//     }

//     for(auto itr = pub_events.begin(); itr != pub_events.end(); ++itr) {
//         std::cout << "key = " << itr->first           // print key
//                   << ", val = " << itr->second << "\n";    // print value
//     }
// }

void CLeader::GetMessages() {
    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if( !tMsgs.empty()) {
        for(int i = 0; i < tMsgs.size(); i++) {

            size_t index = 0;

            Message msg = Message();
            msg.state = static_cast<RobotState>(tMsgs[i].Data[index++]);
            msg.id = std::to_string(tMsgs[i].Data[index++]); // Only stores number part of the id here
            msg.teamid = tMsgs[i].Data[index++];
            msg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);

            /* Message from chain robot */
            if(msg.state == RobotState::CHAIN) {
                msg.id = 'F' + msg.id;

                /* Store which chain entities the other chain robot is connected to */
                while(tMsgs[i].Data[index] != 255) {    // Check if data exists
                    std::string chainID;
                    chainID += (char)tMsgs[i].Data[index++];            // First char of ID
                    chainID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                    msg.connections.push_back(chainID);
                }
                
                sort(std::begin(msg.connections), std::end(msg.connections));
                chainMsgs.push_back(msg);
            } 
            /* Message from team */
            else if(msg.teamid == teamID) {
                // /* Message from leader */
                // if(msg.state == RobotState::LEADER) {
                //     msg.id = 'L' + msg.id;
                //     msg.hasSeenChain = tMsgs[i].Data[index++];
                //     leaderMsg = msg;
                // } 
                /* Message from follower */
                if(msg.state == RobotState::FOLLOWER) {
                    msg.id = 'F' + msg.id;
                    msg.hasSeenChain = tMsgs[i].Data[index++];
                    teamMsgs.push_back(msg);
                }
            } 
            /* Message from other team */
            else {
                /* Message from other leader */
                if(msg.state == RobotState::LEADER) {
                    msg.id = 'L' + msg.id;
                    otherLeaderMsgs.push_back(msg);   
                }
                /* Message from other follower */
                else if(msg.state == RobotState::FOLLOWER) {
                    msg.id = 'F' + msg.id;
                    otherTeamMsgs.push_back(msg); 
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CLeader::UpdateSensors() {
    /* Combine messages received from its own followers and chain entities */
    std::vector<Message> combinedMsgs(chainMsgs);
    combinedMsgs.insert(std::end(combinedMsgs),
                        std::begin(teamMsgs),
                        std::end(teamMsgs));

    /* Check whether follower or chain is nearby (within threshold) */
    for(int i = 0 ; i < combinedMsgs.size(); i++) {
        Real dist = combinedMsgs[i].direction.Length();
        if(dist < minDistanceFromRobot)
            closeToRobot = true;
    }
}

/****************************************/
/****************************************/

CVector2 CLeader::VectorToWaypoint() {   
     /* Get current position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    std::cout << "pos2d " << pos2d << std::endl;

    /* Check if it is near the waypoint */
    Real dist = (waypoints.front() - pos2d).Length();
    std::cout << "dist " << dist << std::endl;
    if(dist < m_sWaypointTrackingParams.thresRange) {
        /* Delete waypoint from queue */
        waypoints.pop();
    }

    /* Calculate a normalized vector that points to the next waypoint */
    CVector2 cAccum = waypoints.front() - pos2d;

    std::cout << "cAccum: " << cAccum << std::endl;
    std::cout << "angle: " << cAccum.Angle() << std::endl;

    cAccum.Rotate((-cZAngle).SignedNormalize());
    
    std::cout << "cAccum: " << cAccum << std::endl;
    std::cout << "angle: " << cAccum.Angle() << std::endl;

    if(cAccum.Length() > 0.0f) {
        /* Make the vector as long as the max speed */
        cAccum.Normalize();
        cAccum *= m_sWheelTurningParams.MaxSpeed;
    }
    std::cout << "cAccum: " << cAccum << std::endl;
    return cAccum;
}

/****************************************/
/****************************************/

CVector2 CLeader::GetOtherRepulsionVector() {
    CVector2 resVec = CVector2();
    int otherSeen = otherLeaderMsgs.size() + otherTeamMsgs.size() + chainMsgs.size();

    if(otherSeen > 0) {

        int numRepulse = 0;

        for(int i = 0; i < otherLeaderMsgs.size(); i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJones(otherLeaderMsgs[i].direction.Length());
            /* Sum to accumulator */
            CVector2 force = CVector2(fLJ,
                                      otherLeaderMsgs[i].direction.Angle());
            /* Only apply repulsive force */
            if( Abs(otherLeaderMsgs[i].direction.Angle() - force.Angle()) >= CRadians::PI_OVER_TWO ) {
                resVec += force;
                numRepulse++;
            }
        }

        for(int i = 0; i < otherTeamMsgs.size(); i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJones(otherTeamMsgs[i].direction.Length());
            /* Sum to accumulator */
            CVector2 force = CVector2(fLJ,
                                      otherTeamMsgs[i].direction.Angle());
            /* Only apply repulsive force */
            if( Abs(otherTeamMsgs[i].direction.Angle() - force.Angle()) >= CRadians::PI_OVER_TWO ) {
                resVec += force;
                numRepulse++;
            }
        }

        for(int i = 0; i < chainMsgs.size(); i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJones(chainMsgs[i].direction.Length());
            /* Sum to accumulator */
            CVector2 force = CVector2(fLJ,
                                      chainMsgs[i].direction.Angle());
            /* Only apply repulsive force */
            if( Abs(chainMsgs[i].direction.Angle() - force.Angle()) >= CRadians::PI_OVER_TWO ) {
                resVec += force;
                numRepulse++;
            }
        }

        if(numRepulse > 0) {
            /* Divide the accumulator by the number of blobs producing repulsive forces */
            resVec /= numRepulse;
            /* Clamp the length of the vector to the max speed */
            if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
                resVec.Normalize();
                resVec *= m_sWheelTurningParams.MaxSpeed;
            }
        }
    }
    return resVec;
}

/****************************************/
/****************************************/

void CLeader::SetWheelSpeedsFromVector(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

    /* Turning state switching conditions */
    if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
        /* No Turn, heading angle very small */
        m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
    }
    else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
        /* Hard Turn, heading angle very large */
        m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
    }
    else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN &&
            Abs(cHeadingAngle) > m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
        /* Soft Turn, heading angle in between the two cases */
        m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
    }

    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(m_sWheelTurningParams.TurningMechanism) {
        case SWheelTurningParams::NO_TURN: {
            /* Just go straight */
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            break;
        }

        case SWheelTurningParams::SOFT_TURN: {
            /* Both wheels go straight, but one is faster than the other */
            Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            break;
        }

        case SWheelTurningParams::HARD_TURN: {
            /* Opposite wheel speeds */
            fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
            fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
            break;
        }
    }

    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if(cHeadingAngle > CRadians::ZERO) {
        /* Turn Left */
        fLeftWheelSpeed  = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else {
        /* Turn Right */
        fLeftWheelSpeed  = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CLeader::SetWheelSpeedsFromVectorHoming(const CVector2& c_heading) {

    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    
    /* Calculate the amount to adjust the wheel speeds */
    Real fSpeed = PIDHeading->calculate(0,cHeadingAngle.GetValue());
    std::cout << fSpeed << std::endl;

    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    fLeftWheelSpeed  = m_sWheelTurningParams.MaxSpeed+fSpeed;
    fRightWheelSpeed = m_sWheelTurningParams.MaxSpeed-fSpeed;

    /* Clamp the speed so that it's not greater than MaxSpeed */
    fLeftWheelSpeed = Min<Real>(fLeftWheelSpeed, m_sWheelTurningParams.MaxSpeed);
    fRightWheelSpeed = Min<Real>(fRightWheelSpeed, m_sWheelTurningParams.MaxSpeed);

    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CLeader::PrintName() {
    std::cout << "[" << this->GetId() << "] ";
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CLeader, "leader_controller")
