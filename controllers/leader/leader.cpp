/* Include the controller definition */
#include "leader.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <utility/team_color.h>
#include <algorithm>

/****************************************/
/****************************************/

static const std::vector<CRadians> PROX_ANGLE {
                                                CRadians::PI / 10.5884f,
                                                CRadians::PI / 3.5999f,
                                                CRadians::PI_OVER_TWO,  // side sensor
                                                CRadians::PI / 1.2f,    // back sensor
                                                CRadians::PI / 0.8571f, // back sensor
                                                CRadians::PI / 0.6667f, // side sensor
                                                CRadians::PI / 0.5806f,
                                                CRadians::PI / 0.5247f
                                              };

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

/*
 * This function is a generalization of the Lennard-Jones potential for repulsion only 
 */
Real CLeader::SFlockingInteractionParams::GeneralizedLennardJonesRepulsion(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp);
}

/****************************************/
/****************************************/

/* 
* Checks whethe the Message is empty or not by checking the direction it was received from
*/
bool CLeader::Message::Empty() {
    return direction.Length() == 0.0f;
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
    m_bSignal(false),
    PIDHeading(NULL),
    nearRobot(false) {}

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
        /* Minimum duration the accept message will be sent for */
        GetNodeAttribute(GetNode(t_node, "timeout"), "send_accept", sendAcceptDuration);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Get team ID from leader ID */
    teamID = stoi(GetId().substr(1));

    /* Initialization */
    currentState = RobotState::LEADER;
    shareToTeam = "";
    initStepTimer = 0;

    /* Set LED color */
    m_pcLEDs->SetAllColors(teamColor[teamID]);

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

CLeader::~CLeader() {
    delete PIDHeading;
}

/****************************************/
/****************************************/

void CLeader::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    msg = CByteArray(91, 255);
    m_pcRABAct->SetData(msg);
    msg_index = 0;

    /* Reset the incoming public events */
    // pub_events.clear();
    // pub_events[EV_b] = false;
}

/****************************************/
/****************************************/

void CLeader::ControlStep() {

    std::cout << "\n---------- " << this->GetId() << " ----------" << std::endl;

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/

    /* Create new message */
    msg = CByteArray(91, 255);
    msg_index = 0;

    /* Clear messages received */
    teamMsgs.clear();
    connectorMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();

    cmsgToSend.clear();

    nearRobot = false;

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
    Update();
    
    /* Set its state in msg */
    msg[msg_index++] = static_cast<UInt8>(currentState);
    /* Set sender ID in msg */
    msg[msg_index++] = teamID;  // For leader, ID = teamID
    /* Set team ID in msg */
    msg[msg_index++] = teamID;

    /*---------*/
    /* Control */
    /*---------*/

    if(initStepTimer > 1) {

        /* Is the robot selected? */
        if(m_bSelected) {

            /* Follow the control vector */
            SetWheelSpeedsFromVector(m_cControl);
            std::cout << "SIGNAL " << m_bSignal << std::endl; 

            if(m_bSignal)
                m_pcLEDs->SetAllColors(CColor::WHITE);
            else
                m_pcLEDs->SetAllColors(teamColor[teamID]);

            msg[msg_index++] = int(m_bSignal); // Set the signal the leader is sending
        }
        else {
            if( !nearRobot ) {
                std::cout << "[LOG] Robot is far" << std::endl;

                /* Stop if other robots are too far from itself */
                m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
                msg_index++;
            }
            else if( !waypoints.empty() ) {
                /* Check if it is near the waypoint */
                CVector3 pos3d = m_pcPosSens->GetReading().Position;
                CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
                Real dist = (waypoints.front() - pos2d).Length();
                std::cout << "dist: " << dist << std::endl;

                /* Check if task is completed */
                if(dist < m_sWaypointTrackingParams.thresRange) {
                    if(currentTaskDemand == 0) {
                        msg[msg_index++] = 0; // send stopTask signal to robots in the team
                        m_pcLEDs->SetAllColors(teamColor[teamID]);
                        waypoints.pop(); // Delete waypoint from queue
                    } else {
                        msg[msg_index++] = 1; // send startTask signal to robots in the team
                        m_pcLEDs->SetAllColors(CColor::WHITE);
                    }
                } else
                    msg[msg_index++] = 0; // Leader is not close to a waypoint

                /* If current task is completed, move to the next one */
                if(dist > m_sWaypointTrackingParams.thresRange || currentTaskDemand == 0) {
                    
                    std::cout << "[LOG] Moving to next task" << std::endl;

                    /* Calculate overall force applied to the robot */
                    CVector2 waypointForce = VectorToWaypoint();           // Attraction to waypoint
                    CVector2 robotForce    = GetRobotRepulsionVector();    // Repulsion from other robots
                    CVector2 obstacleForce = GetObstacleRepulsionVector(); // Repulsion from obstacles

                    CVector2 sumForce      = waypointForce + robotForce + obstacleForce;
                    std::cout << "waypointForce: " << waypointForce << std::endl;
                    std::cout << "robotForce: " << robotForce << std::endl;
                    std::cout << "obstacleForce: " << obstacleForce << std::endl;
                    std::cout << "sumForce: " << sumForce << std::endl;

                    SetWheelSpeedsFromVectorHoming(sumForce);
                } 
                else {
                    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
                }
            }
            else {
                std::cout << "[LOG] No assigned tasks left" << std::endl;
                m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
                msg[msg_index++] = 0; // Leader sends stopTask
            }
        }
    } else {
        initStepTimer++;
        msg_index++;
    }
    
    /* Hop count */
    msg[msg_index++] = 1; // Number of HopMsg

    msg[msg_index++] = teamID;
    msg[msg_index++] = 0; // Hop count
    msg_index += 2; // Skip ID

    msg_index += 4; // Skip to next part

    /* Connection Message */
    std::cout << "cmsgToSend.size: " << cmsgToSend.size() << std::endl;
    msg[msg_index++] = cmsgToSend.size(); // Set the number of ConnectionMsg
    for(const auto& conMsg : cmsgToSend) {
        msg[msg_index++] = (UInt8)conMsg.type;
        msg[msg_index++] = conMsg.from[0];
        msg[msg_index++] = stoi(conMsg.from.substr(1));
        msg[msg_index++] = conMsg.to[0];
        msg[msg_index++] = stoi(conMsg.to.substr(1));
        msg[msg_index++] = conMsg.toTeam;
    }
    // Skip if not all bytes are used
    msg_index += (2 - cmsgToSend.size()) * 6; // TEMP: Currently assuming only two teams

    /* Update Message */
    msg_index += 2; // Skip shareToLeader

    if( !shareToTeam.empty() ) {
        msg[msg_index++] = shareToTeam[0];
        msg[msg_index++] = stoi(shareToTeam.substr(1));
    } else
        msg_index += 2;

    /* Set ID of all connections to msg */
    std::vector<Message> allMsgs(teamMsgs);
    allMsgs.insert(std::end(allMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

    for(size_t i = 0; i < allMsgs.size(); i++) {
        msg[msg_index++] = allMsgs[i].ID[0];    // First character of ID
        msg[msg_index++] = stoi(allMsgs[i].ID.substr(1));    // ID number

        if(i >= 30)
            break;
    }

    /*--------------*/
    /* Send message */
    /*--------------*/
    m_pcRABAct->SetData(msg);

    /* Reset task demand */
    currentTaskDemand = 0;

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

void CLeader::SetSignal(const bool b_signal) {
    m_bSignal = b_signal;
}

/****************************************/
/****************************************/

CVector2 CLeader::GetNextWaypoint() {
    return waypoints.front();
}

/****************************************/
/****************************************/

void CLeader::SetWaypoints(const std::queue<CVector2> waypts) {
    waypoints = waypts;
}

/****************************************/
/****************************************/

void CLeader::SetTaskDemand(const UInt32 un_demand) {
    currentTaskDemand = un_demand;
}

/****************************************/
/****************************************/

void CLeader::GetMessages() {
    
    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if( !tMsgs.empty() ) {
        for(int i = 0; i < tMsgs.size(); i++) {

            std::cout << tMsgs[i].Data << std::endl;

            size_t index = 0;

            Message msg = Message();

            /* Core */
            msg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);
            msg.state = static_cast<RobotState>(tMsgs[i].Data[index++]);
            msg.ID = std::to_string(tMsgs[i].Data[index++]); // Only stores number part of the id here
            msg.teamID = tMsgs[i].Data[index++];

            /* Leader Signal */
            msg.leaderSignal = tMsgs[i].Data[index++];

            /* Hops */
            UInt8 msg_num = tMsgs[i].Data[index++];

            if(msg_num == 255) // Safety check value
                msg_num = 0;

            for(size_t j = 0; j < msg_num; j++) {

                HopMsg hop;

                UInt8 tmpTeamID = tMsgs[i].Data[index++];
                hop.count = tMsgs[i].Data[index++];

                if(hop.count > 1) {
                    std::string robotID;
                    robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                    robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                    hop.ID = robotID;
                } else
                    index += 2;
                
                msg.hops[tmpTeamID] = hop;
            }
            index += (2 - msg_num) * 4; // TEMP: Currently assuming only two teams

            /* Connection Message */
            msg_num = tMsgs[i].Data[index++];

            if(msg_num == 255)
                msg_num = 0;

            for(size_t j = 0; j < msg_num; j++) {

                ConnectionMsg conMsg;

                conMsg.type = (char)tMsgs[i].Data[index++];

                std::string robotID;
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                conMsg.from = robotID;

                std::cout << "FROM: " << conMsg.from << std::endl;

                robotID = "";
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                conMsg.to = robotID;
                
                std::cout << "TO: " << conMsg.to << std::endl;
                
                conMsg.toTeam = tMsgs[i].Data[index++]; 

                msg.cmsg.push_back(conMsg);
            }
            index += (2 - msg_num) * 6; // TEMP: Currently assuming only two teams
            
            /* Update Message */
            std::string robotID;
            if(tMsgs[i].Data[index] == 255) {
                index += 2;
            } else {
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                msg.shareToLeader = robotID;
            }
            
            std::cout << "shareToLeader: " << msg.shareToLeader << std::endl;

            if(tMsgs[i].Data[index] == 255) {
                index += 2;
            } else {
                robotID = "";
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                msg.shareToTeam = robotID;
            }

            std::cout << "shareToTeam: " << msg.shareToTeam << std::endl;

            /* Connections */
            while(tMsgs[i].Data[index] != 255) {    // Check if data exists
                std::string robotID;
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                msg.connections.push_back(robotID);
            }

            /* Store message */
            if(msg.state == RobotState::LEADER) {
                msg.ID = 'L' + msg.ID;
                otherLeaderMsgs.push_back(msg);

            } else if(msg.state == RobotState::FOLLOWER) {
                msg.ID = 'F' + msg.ID;

                if(msg.teamID == teamID)
                    teamMsgs.push_back(msg);
                else
                    otherTeamMsgs.push_back(msg);

            } else if(msg.state == RobotState::CONNECTOR) {
                msg.ID = 'F' + msg.ID;
                connectorMsgs.push_back(msg);
            }
        }
    }
}

/****************************************/
/****************************************/

void CLeader::Update() {

    nearRobot = IsNearRobot();

    SetConnectorToRelay();

    ReplyToRequest();

    /* Set ConnectionMsg to send during this timestep */
    std::cout << "resend size: " << cmsgToResend.size() << std::endl;
    for(auto it = cmsgToResend.begin(); it != cmsgToResend.end();) {
        if(it->first > 0) {
            cmsgToSend.push_back(it->second);
            it->first--; // Decrement timer
            ++it;
        } else {
            it = cmsgToResend.erase(it);
        }
    }
}

/****************************************/
/****************************************/

bool CLeader::IsNearRobot() {
    
    /* Combine messages received */
    std::vector<Message> combinedMsgs(connectorMsgs);
    combinedMsgs.insert(std::end(combinedMsgs),
                        std::begin(teamMsgs),
                        std::end(teamMsgs));
    combinedMsgs.insert(std::end(combinedMsgs),
                        std::begin(otherLeaderMsgs),
                        std::end(otherLeaderMsgs));
    combinedMsgs.insert(std::end(combinedMsgs),
                        std::begin(otherTeamMsgs),
                        std::end(otherTeamMsgs));

    /* Check whether there is a neighbor (within threshold) */
    for(int i = 0 ; i < combinedMsgs.size(); i++) {
        Real dist = combinedMsgs[i].direction.Length();
        if(dist < minDistanceFromRobot)
            return true;
    }
    return false;
}

/****************************************/
/****************************************/

void CLeader::ReplyToRequest() {

    /* Upon receiving a request message, send an accept message to the follower with the smallest ID */
    ConnectionMsg acceptTo;
    acceptTo.type   = 'A';
    acceptTo.from   = this->GetId();
    acceptTo.toTeam = teamID;
    
    for(const auto& teamMsg : teamMsgs) {
        for(const auto& cmsg : teamMsg.cmsg) {

            if(cmsg.to == this->GetId() && cmsg.type == 'R' && shareToTeam.empty()) {

                /* Set the ID of the first follower request seen */
                if(acceptTo.to.empty()) {
                    acceptTo.to = cmsg.from;
                    continue;
                }
                
                UInt8 currentFID = stoi(acceptTo.to.substr(1));
                UInt8 newFID = stoi(cmsg.from.substr(1));

                /* Send an accept message to the follower with the smallest ID */
                if(newFID < currentFID)
                    acceptTo.to = cmsg.from;
            }
        }
    }

    std::cout << "acceptTo: " << acceptTo.to << std::endl;

    /* Add accept message to be sent */
    if( !acceptTo.to.empty() ) {
        // cmsgToSend.push_back(acceptTo);
        cmsgToResend.push_back({sendAcceptDuration,acceptTo});
    }
}

/****************************************/
/****************************************/

void CLeader::SetConnectorToRelay() {

    /* Check if a connector with a hop count = 1 to its current team is nearby */
    bool foundFirstConnector = false;
    for(const auto& msg : connectorMsgs) {
        auto hopInfo = msg.hops;

        if(hopInfo[teamID].count == 1) {

            /* Send info of connector found */
            shareToTeam = msg.ID;
            foundFirstConnector = true;
            break;
        }
    }

    /* If the first connector is not nearby, check which it should relay upstream */
    if( !foundFirstConnector ) {
        if( !teamMsgs.empty() ) {

            bool previousSeen = false;
            bool newValue = false;

            for(const auto& msg : teamMsgs) {

                if(msg.shareToLeader == shareToTeam)
                    previousSeen = true; // If only same info, send the same connector
                else if( !msg.shareToLeader.empty() ) {

                    /* Update to connector info that's different from previous */
                    shareToTeam = msg.shareToLeader;
                    newValue = true;
                    break;
                }
            }

            if( !previousSeen && !newValue ) // If previous info not received and no new info, send nothing
                shareToTeam = "";
            
        } else // If no upstream exists, send nothing
            shareToTeam = "";
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

CVector2 CLeader::GetRobotRepulsionVector() {
    CVector2 resVec = CVector2();
    int otherSeen = otherLeaderMsgs.size() + otherTeamMsgs.size() + connectorMsgs.size();

    if(otherSeen > 0) {

        int numRepulse = 0;

        for(int i = 0; i < otherLeaderMsgs.size(); i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJonesRepulsion(otherLeaderMsgs[i].direction.Length());
            /* Sum to accumulator */
            resVec += CVector2(fLJ,
                               otherLeaderMsgs[i].direction.Angle());
            numRepulse++;
        }

        for(int i = 0; i < otherTeamMsgs.size(); i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJonesRepulsion(otherTeamMsgs[i].direction.Length());
            /* Sum to accumulator */
            resVec += CVector2(fLJ,
                               otherTeamMsgs[i].direction.Angle());
            numRepulse++;
        }

        for(int i = 0; i < connectorMsgs.size(); i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJonesRepulsion(connectorMsgs[i].direction.Length());
            /* Sum to accumulator */
            resVec += CVector2(fLJ,
                               connectorMsgs[i].direction.Angle());
            numRepulse++;
        }

        if(numRepulse > 0) {
            /* Divide the accumulator by the number of e-pucks producing repulsive forces */
            resVec /= numRepulse;
            /* Clamp the length of the vector to half of the max speed */
            if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
                resVec.Normalize();
                resVec *= m_sWheelTurningParams.MaxSpeed * 0.5;
            }
        }
    }
    return resVec;
}

/****************************************/
/****************************************/

CVector2 CLeader::GetObstacleRepulsionVector() {
    /* Get proximity sensor readings */
    std::vector<Real> fProxReads = m_pcProximity->GetReadings();

    CVector2 resVec = CVector2();

    for(size_t i = 0; i < fProxReads.size(); i++) {
        CVector2 vec = CVector2();
        if(fProxReads[i] > 0.0f) {
            Real length = (fProxReads[i] - 0.9) * m_sWheelTurningParams.MaxSpeed * 10; // Map length to 0 ~ max_speed
            vec = CVector2(length, PROX_ANGLE[i]);

            resVec -= vec; // Subtract because we want the vector to repulse from the obstacle
        }
        // std::cout << "sensor " << i << ": " << vec.Length() << std::endl;
    }

    /* Clamp the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
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
