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

CLeader::CLeader() :
    m_pcWheels(NULL),
    m_pcProximity(NULL) {}

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
        /* Goal allowance range */
        GetNodeAttribute(GetNode(t_node, "path_following"), "goal_range", goalRange);
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
    // teamMsgs.clear();
    // chainMsgs.clear();
    // otherLeaderMsgs.clear();
    // otherTeamMsgs.clear();

    // for(int i = 0; i < waypoints.size(); i++) {
    //     std::cout << waypoints[i].GetX() << "," << waypoints[i].GetY() << std::endl;
    // }

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();

    /* Create new message */
    msg = CByteArray(16, 255);
    msg_index = 0;
    /* Set its state in msg */
    msg[msg_index++] = static_cast<UInt8>(currentState);
    /* Set sender ID in msg */
    std::string id = this->GetId();
    msg[msg_index++] = stoi(id.substr(1));  // For leader, ID = teamID
    /* Set team ID in msg */
    msg[msg_index++] = teamID;
    /* Set whether it has seen a chain */
    // if( !chainMsgs.empty() || !otherLeaderMsgs.empty() )
    //     msg[msg_index++] = 1;
    // else
    //     msg[msg_index++] = 0;
    msg[msg_index++] = 1;

    /*---------*/
    /* Control */
    /*---------*/

    /* Follow the control vector only if selected */
    if(m_bSelected)
        SetWheelSpeedsFromVector(m_cControl);
    else {
        if( !waypoints.empty() )
            SetWheelSpeedsFromVector(VectorToWaypoint());
        else
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    }

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
    // /* Get RAB messages from nearby e-pucks */
    // const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    // if( !tMsgs.empty()) {
    //     for(int i = 0; i < tMsgs.size(); i++) {

    //         size_t index = 0;

    //         Message msg = Message();
    //         msg.state = static_cast<RobotState>(tMsgs[i].Data[index++]);
    //         msg.id = std::to_string(tMsgs[i].Data[index++]); // Only stores number part of the id here
    //         msg.teamid = tMsgs[i].Data[index++];
    //         msg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);

    //         /* Message from chain robot */
    //         if(msg.state == RobotState::CHAIN) {
    //             msg.id = 'F' + msg.id;

    //             /* Store which chain entities the other chain robot is connected to */
    //             while(tMsgs[i].Data[index] != 255) {    // Check if data exists
    //                 std::string chainID;
    //                 chainID += (char)tMsgs[i].Data[index++];            // First char of ID
    //                 chainID += std::to_string(tMsgs[i].Data[index++]);  // ID number
    //                 msg.connections.push_back(chainID);
    //             }
                
    //             sort(std::begin(msg.connections), std::end(msg.connections));
    //             chainMsgs.push_back(msg);
    //         } 
    //         /* Message from team */
    //         else if(msg.teamid == teamID) {
    //             // /* Message from leader */
    //             // if(msg.state == RobotState::LEADER) {
    //             //     msg.id = 'L' + msg.id;
    //             //     msg.hasSeenChain = tMsgs[i].Data[index++];
    //             //     leaderMsg = msg;
    //             // } 
    //             /* Message from follower */
    //             if(msg.state == RobotState::FOLLOWER) {
    //                 msg.id = 'F' + msg.id;
    //                 msg.hasSeenChain = tMsgs[i].Data[index++];
    //                 teamMsgs.push_back(msg);
    //             }
    //         } 
    //         /* Message from other team */
    //         else {
    //             /* Message from other leader */
    //             if(msg.state == RobotState::LEADER) {
    //                 msg.id = 'L' + msg.id;
    //                 otherLeaderMsgs.push_back(msg);   
    //             }
    //             /* Message from other follower */
    //             else if(msg.state == RobotState::FOLLOWER) {
    //                 msg.id = 'F' + msg.id;
    //                 otherTeamMsgs.push_back(msg); 
    //             }
    //         }
    //     }
    // }
}

/****************************************/
/****************************************/

void CLeader::UpdateSensors() {}

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
    if(dist < goalRange) {
        /* Delete waypoint from queue */
        waypoints.pop();
    }

    /* Calculate a normalized vector that points to the next waypoint */
    // CRadians angle = cZAngle - waypoints.front().Angle();

    CVector2 cAccum = waypoints.front() - pos2d;
    std::cout << "cAccum: " << cAccum << std::endl;
    std::cout << "angle: " << cAccum.Angle() << std::endl;
    cAccum.Rotate((-cZAngle).SignedNormalize());
    
    std::cout << "cAccum: " << cAccum << std::endl;
    std::cout << "angle: " << cAccum.Angle() << std::endl;

    // std::cout << "cZAngle: " << cZAngle << std::endl;
    // std::cout << "Waypts: " << waypoints.front().Angle() << std::endl;
    // std::cout << "angle: " << angle << std::endl;
    // CVector2 cAccum = CVector2(dist, angle);
    // std::cout << res << std::endl;
    // std::cout << cAccum.Length() << ", " << 

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
