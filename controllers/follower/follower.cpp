/* Include the controller definition */
#include "follower.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <utility/team_color.h>
#include <algorithm>
#include <set>

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

void CFollower::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

void CFollower::SLeaderInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "kp", Kp);
      GetNodeAttribute(t_node, "ki", Ki);
      GetNodeAttribute(t_node, "kd", Kd);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFollower::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
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
Real CFollower::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential for repulsion only 
 */
Real CFollower::SFlockingInteractionParams::GeneralizedLennardJonesRepulsion(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp);
}

/****************************************/
/****************************************/

CFollower::CFollower() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_pcLEDs(NULL),
    sct(NULL),
    pid(NULL) {}

/****************************************/
/****************************************/

CFollower::~CFollower() {
    delete sct;
    delete pid;
}

/****************************************/
/****************************************/

void CFollower::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"            );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );

    /*
    * Parse the config file
    */
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

        /* Flocking-related */
        m_sLeaderFlockingParams.Init(GetNode(t_node, "leader_flocking"));
        m_sTeamFlockingParams.Init(GetNode(t_node, "team_flocking"));

        /* Chain formation threshold */
        GetNodeAttribute(GetNode(t_node, "team"), "separation_threshold", separationThres);
        GetNodeAttribute(GetNode(t_node, "team"), "joining_threshold", joiningThres);

        /* Weights for the flocking behavior */
        GetNodeAttribute(GetNode(t_node, "flocking_weights"), "team",     teamWeight);
        GetNodeAttribute(GetNode(t_node, "flocking_weights"), "robot",    robotWeight);
        GetNodeAttribute(GetNode(t_node, "flocking_weights"), "obstacle", obstacleWeight);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Initialization */
    currentState = RobotState::CONNECTOR; // Set initial state to connector
    performingTask = false; // Robot initially not working on any task
    hopCountToLeader = 255; // Default (max) value as hop count is unknown

    /*
    * Init SCT Controller
    */
    sct = new SCT();

    /* Register controllable events */
    sct->add_callback(this, EV_moveFlock, &CFollower::Callback_MoveFlock, NULL, NULL);
    sct->add_callback(this, EV_moveStop,  &CFollower::Callback_MoveStop,  NULL, NULL);
    sct->add_callback(this, EV_setF,      &CFollower::Callback_SetF,      NULL, NULL);
    sct->add_callback(this, EV_setC,      &CFollower::Callback_SetC,      NULL, NULL);
    sct->add_callback(this, EV_sendR,     &CFollower::Callback_SendR,     NULL, NULL);
    sct->add_callback(this, EV_sendA,     &CFollower::Callback_SendA,     NULL, NULL);
    
    /* Register uncontrollable events */
    // sct->add_callback(this, EV_condF1,    NULL, &CFollower::Check_CondF1,    NULL);
    // sct->add_callback(this, EV_notCondF1, NULL, &CFollower::Check_NotCondF1, NULL);
    // sct->add_callback(this, EV_condF2,    NULL, &CFollower::Check_CondF2,    NULL);
    // sct->add_callback(this, EV_notCondF2, NULL, &CFollower::Check_NotCondF2, NULL);
    sct->add_callback(this, EV_condC1,    NULL, &CFollower::Check_CondC1,    NULL);
    sct->add_callback(this, EV_notCondC1, NULL, &CFollower::Check_NotCondC1, NULL);
    sct->add_callback(this, EV_condC2,    NULL, &CFollower::Check_CondC2,    NULL);
    sct->add_callback(this, EV_notCondC2, NULL, &CFollower::Check_NotCondC2, NULL);
    sct->add_callback(this, EV_condC3,    NULL, &CFollower::Check_CondC3,    NULL);
    sct->add_callback(this, EV_notCondC3, NULL, &CFollower::Check_NotCondC3, NULL);

    /*
    * Init PID Controller
    */
    pid = new PID(0.1,  // dt  (loop interval time)
                  80,   // max (output vector length)
                  -80,  // min (output vector length)
                  m_sLeaderFlockingParams.Kp,    // Kp
                  m_sLeaderFlockingParams.Ki,    // Ki
                  m_sLeaderFlockingParams.Kd);   // Kd

    Reset();
}

/****************************************/
/****************************************/

void CFollower::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    msg = CByteArray(85, 255);
    m_pcRABAct->SetData(msg);
    msg_index = 0;

}

/****************************************/
/****************************************/

void CFollower::ControlStep() {

    std::cout << "\n---------- " << this->GetId() << " ----------" << std::endl;

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/

    /* Create new msg */
    msg = CByteArray(85, 255);
    msg_index = 0;

    /* Clear messages received */
    leaderMsg = Message();
    teamMsgs.clear();
    connectorMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();

    leaderSignal = 255; // Default value for no signal
    hopCountToLeader = 255; // Default value for not known hop count to the leader
    cmsg = ConnectionMsg();

    /* Reset sensor reading results */
    condC2 = true;

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();

    /*------------------------*/
    /* Update sensor readings */
    /*------------------------*/
    UpdateSensors();

    // /*--------------------*/
    // /* Run SCT controller */
    // /*--------------------*/
    sct->run_step();

    sct->print_current_state();
    std::cout << std::endl;

    /*-----------------------------*/
    /* Implement action to perform */
    /*-----------------------------*/

    /* Set current state in msg */
    msg[msg_index++] = static_cast<UInt8>(currentState);

    /* Set sender ID in msg */
    std::string id = this->GetId();
    msg[msg_index++] = stoi(id.substr(1));

    /* Set current team ID in msg */
    msg[msg_index++] = teamID;

    // Decide what to communicate depending on current state (switch between follower and connector)
    switch(currentState) {
        case RobotState::FOLLOWER: {
            std::cout << "State: FOLLOWER" << std::endl;
            m_pcLEDs->SetAllColors(teamColor[teamID]);

            /* Relay leader signal */
            msg[msg_index++] = leaderSignal;

            /* Set its hop count to the leader */
            std::cout << "HOP = " << hopCountToLeader << std::endl;

            /* Hop count */
            msg[msg_index++] = 1; // Number of HopMsg

            msg[msg_index++] = hopCountToLeader;
            msg_index += 2; // Skip ID
            msg[msg_index++] = teamID;

            msg_index += 4; // Skip to next part

            std::cout << "type: " << cmsg.type << std::endl;

            /* Connection message */
            if(cmsg.type != 'N') {
                msg[msg_index++] = 1; // Number of ConnectionMsg
                msg[msg_index++] = (UInt8)cmsg.type;
                msg[msg_index++] = cmsg.to[0];
                msg[msg_index++] = cmsg.to[1];
                msg[msg_index++] = cmsg.from[0];
                msg[msg_index++] = cmsg.from[1];
            } else {
                msg[msg_index++] = 0; // Number of ConnectionMsg
                msg_index += 5;
            }

            msg_index += 5; // Skip to next part

            break;
        }
    //     case RobotState::CONNECTOR: {
    //         std::cout << "State: CONNECTOR" << std::endl;
    //         m_pcLEDs->SetAllColors(CColor::CYAN);

    //         /* Leader signal */
    //         msg_index++; // Skip to next part

    //         /* Hop count */
    //         msg_index += 9; // TEMP: Skip to next part

    //         /* Connection Message */
    //         msg_index += 11; // TEMP: Skip to next part

    //         break;
    //     }
        case RobotState::LEADER: {
            std::cout << "State: LEADER. Something went wrong." << std::endl;
            break;
        }
    }

    // switch(currentMoveType) {
    //     case MoveType::FLOCK: {
    //         Flock();
    //         break;
    //     }
    //     case MoveType::STOP: {
    //         m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    //         break;
    //     }
    // }

    /* Set ID of all connections to msg */
    std::vector<Message> allMsgs(teamMsgs);
    allMsgs.insert(std::end(allMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

    if(leaderMsg.direction.Length() > 0.0f) {
        allMsgs.push_back(leaderMsg);
    }

    std::cout << "I saw: ";
    for(size_t i = 0; i < allMsgs.size(); i++) {
        std::cout << allMsgs[i].ID << ", ";

        msg[msg_index++] = allMsgs[i].ID[0];    // First character of ID
        msg[msg_index++] = stoi(allMsgs[i].ID.substr(1));    // ID number

        if(i >= 30)
            break;
    }
    std::cout << std::endl;

    /*--------------*/
    /* Send message */
    /*--------------*/
    m_pcRABAct->SetData(msg);

}

/****************************************/
/****************************************/

void CFollower::GetMessages() {

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

                hop.count = tMsgs[i].Data[index++];

                std::string robotID;
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                hop.ID = robotID;

                hop.teamID = tMsgs[i].Data[index++];

                msg.hops[msg.teamID] = hop;
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
                conMsg.to = robotID;

                robotID = "";

                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                conMsg.from = robotID;

                msg.cmsg[msg.teamID] = conMsg;
            }
            index += (2 - msg_num) * 5; // TEMP: Currently assuming only two teams
            
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

                if(msg.teamID == teamID)
                    leaderMsg = msg;
                else
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

void CFollower::UpdateSensors() {

    std::cout << "connectorMsg  = " << connectorMsgs.size() << std::endl;
    std::cout << "otherLMsg = " << otherLeaderMsgs.size() << std::endl;

    /* Combine messages received from all non-team entities */
    std::vector<Message> nonTeamMsgs(connectorMsgs);
    // nonTeamMsgs.insert(std::end(nonTeamMsgs),
    //                    std::begin(otherLeaderMsgs),
    //                    std::end(otherLeaderMsgs));
    nonTeamMsgs.insert(std::end(nonTeamMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

    if(currentState == RobotState::FOLLOWER) {

        /* Check the closest non team member that could potentially connect */
        Real minDist = 255;
        for(size_t i = 0; i < nonTeamMsgs.size(); i++) {
            Real dist = nonTeamMsgs[i].direction.Length();
            std::cout << "dist " << dist << std::endl;
            if(dist < minDist) {
                minDist = dist;
                connectionCandidate = nonTeamMsgs[i];
            }
        }

        // TODO: Prioritize connectors. Look for other follower if no connectors in range

        std::cout << "Dist to candidate: " << minDist << std::endl;

        /* Find the hop count to and signal from the leader */
        if(leaderMsg.direction.Length() > 0.0f) { // Leader is in range

            hopCountToLeader = 1;
            leaderSignal = leaderMsg.leaderSignal;

        } else { // Leader is not in range. Relay signal

            UInt8 minCount = 255;

            /* Find the smallest hop count among team members */
            for(size_t i = 0; i < teamMsgs.size(); i++) {
                if(teamMsgs[i].hops[teamID].count < minCount) 
                    minCount = teamMsgs[i].hops[teamID].count;
            }

            /* Record its own hop count */
            if(minCount < 255)
                hopCountToLeader = minCount + 1; // Set its count to +1 the smallest value

            for(size_t i = 0; i < teamMsgs.size(); i++) {

                if(teamMsgs[i].hops[0].count < hopCountToLeader) { // Follower will only have one HopMsg so read the first item
                    leaderSignal = teamMsgs[i].leaderSignal;
                    break;
                }
            }
        }

        // Check whether it is the closest to the candidate among other followers in the team that sees it (condC2)
            // Assume it is the closest to the candidate
            // Loop teamMsgs
                // If another follower is closer to the candidate AND it sees the follower (Loop connections)
                    // not closest = condC2 is false

        /* Check condC2 */
        // Is this robot closest to the previously found candidate robot?


        // // Identify ConnectionMsg to relay
        //     // Loop teamMsgs
        //         // If message is accept and hop count is smaller, choose that
        //             // break
        //         // If message is request and hop count is larger, choose that
        //         // If message is update and hop count is larger, choose that
        
        /* Identify ConnectionMsg to send/relay */
        // std::vector<Message> combinedTeamMsgs(teamMsgs);
        // combinedTeamMsgs.push_back(leaderMsg);

        // for(size_t i = 0; i < combinedTeamMsgs.size(); i++) {

        //     Message msg = combinedTeamMsgs[i];
        //     ConnectionMsg conMsg = msg.cmsg[0]; // Leader/Follower will only have one ConnectionMsg so read the first item
            
        //     if(conMsg.type == 'A' && msg.hops[0].count < hopCountToLeader) {
        //         cmsg = conMsg;
        //         break;
        //     } 
            
        //     if(msg.hops[0].count > hopCountToLeader) {

        //         std::string leaderID = "L" + std::to_string(teamID);

        //         // Relay request if the type = 'R' and it is directed to the leader
        //         if(conMsg.type == 'R' && conMsg.to == leaderID)
        //             cmsg = conMsg;
        //     }
        // }

        /* If Accept or Request message was not present, relay an update message */
        // if(cmsg.type == 255) {

        //     for(size_t i = 0; i < teamMsgs.size(); i++) {
        //         if(teamMsgs[i].hops[0].count > hopCountToLeader)
        //     }
        // }




        // Send request (in callback)
            // Create ConnectionMsg
            // Overwrite ConnectionMsg


        // std::vector<Message> combinedTeamMsgs(teamMsgs);
        // combinedTeamMsgs.push_back(leaderMsg);

        // // Initialize variables with its own values
        // myClosest.state = connectionCandidate.state;
        // myClosest.dist = minDist;
        // myClosest.ID = this->GetId();
        // myClosest.timestamp = 0;

        // std::cout << "closeDist " << myClosest.dist << std::endl;
        // std::cout << "closeID " << myClosest.ID << std::endl;

        // std::unordered_map<std::string, ClosestInfo> closestFollowers;

        // // For each message, store info in a map using its ID as key
        // // Overwrite if message with newer timestamp exists
        // for(size_t i = 0; i < combinedTeamMsgs.size(); i++) {
            
        //     std::string id = combinedTeamMsgs[i].closest.ID;

        //     /* Store if key does not exist OR there is a newer one*/
        //     if(closestFollowers.find(id) == closestFollowers.end())
        //         closestFollowers[id] = combinedTeamMsgs[i].closest;
        //     else if(combinedTeamMsgs[i].closest.timestamp < closestFollowers[id].timestamp)
        //         closestFollowers[id] = combinedTeamMsgs[i].closest;
        // }

        // Now iterate through the map to compare the distance







        /* Check the team's closest non team robot */
        // for(size_t i = 0; i < combinedTeamMsgs.size(); i++) {
        //     std::cout << "i: " << i << std::endl;

        //     if( !combinedTeamMsgs[i].closest.ID.empty() ) {    // Check if entry exists

        //         /* Store the info of the robot with the shortest distance */
        //         if(combinedTeamMsgs[i].closest.dist < round(myClosest.dist)) {
        //             condC2 = false;
        //             myClosest = combinedTeamMsgs[i].closest;
        //         }
        //         else if(combinedTeamMsgs[i].closest.dist == round(myClosest.dist)) {
                    
        //             /* If distance is the same, store the info of robot with the smaller ID */
        //             if(stoi(combinedTeamMsgs[i].closest.ID.substr(1)) < stoi(myClosest.ID.substr(1))) {
        //                 condC2 = false;
        //                 myClosest = combinedTeamMsgs[i].closest;
        //             }
        //         }
        //     }
        //     std::cout << "closeDist " << myClosest.dist << std::endl;
        //     std::cout << "closeID " << myClosest.ID << std::endl;
        // }
    }

    // Receive info about closest non team members from other followers
    // If distance is smaller than its own or distance is same and own ID is larger, continue
        // condC2 is false
        // Choose the robot with the smallest distance and rebroadcast (if identical, compare robot ID)

    // Else
        // condC2 is true
        // Prepare to rebroadcast itself



    /* Check if any team member is closer to a non-team member and has seen a non-team member */
    /* Event: isNearest, notNearest */

    // if(currentState == RobotState::FOLLOWER) {

    //     // /* Initializa a list of non team robots that this robot is the closest */
    //     // std::vector<std::string> closestNonTeamRobots; // CURRENTLY CHECKS FOR ONLY ONE FOR OPTIMIZATION

    //     // /* Add leader to team messages*/
    //     // std::vector<Message> combinedTeamMsgs(teamMsgs);
    //     // // if(leaderMsg.direction.Length() > 0.0f)
    //     // //     combinedTeamMsgs.push_back(leaderMsg);

    //     // for(size_t i = 0; i < nonTeamMsgs.size(); i++) {

    //     //     std::cout << "Checking " << nonTeamMsgs[i].id << std::endl;

    //     //     /* Find the distance between itself and the non team robot */
    //     //     Real myDist = nonTeamMsgs[i].direction.Length();

    //     //     /* Flag to check whether there is a team robot closer than itself to a non team robot */
    //     //     bool isClosest = true;

    //     //     for(size_t j = 0; j < combinedTeamMsgs.size(); j++) {
                
    //     //         std::vector<std::string> connections = combinedTeamMsgs[j].connections;

    //     //         std::cout << "(" << combinedTeamMsgs[j].id << ") ";
    //     //         for(size_t k = 0; k < connections.size(); k++)
    //     //             std::cout << connections[k] << ", ";
    //     //         std::cout << std::endl;

    //     //         // Check if the team robot has seen the non-team robot 
    //     //         if (std::find(connections.begin(), connections.end(), nonTeamMsgs[i].id) != connections.end()) {

    //     //             /* Check the distance between the team robot and the non team robot*/
    //     //             CVector2 diff = nonTeamMsgs[i].direction - combinedTeamMsgs[j].direction;
    //     //             Real dist = diff.Length();

    //     //             if(dist < myDist) {
    //     //                 isClosest = false;  // Not the closest to the non team robot
    //     //                 break;
    //     //             }
    //     //         }
    //     //     }
    //     //     if(isClosest) {
    //     //         closestNonTeamRobots.push_back(nonTeamMsgs[i].id);
    //     //         std::cout << "Closest to: " << nonTeamMsgs[i].id << std::endl;
    //     //         break;
    //     //     }
    //     // }

    //     // if(closestNonTeamRobots.empty())
    //     //     isClosestToNonTeam = false;
    //     // else
    //     //     isClosestToNonTeam = true;

    // } 
    // else if(currentState == RobotState::CONNECTOR) {
        
    //     // /* Find unique team IDs detected */
    //     // std::set<UInt8> teamIDs;

    //     // std::cout << "Teams found: ";
    //     // for(size_t i = 0; i < nonTeamMsgs.size(); i++) {
    //     //     teamIDs.insert(nonTeamMsgs[i].teamid);
    //     //     std::cout << nonTeamMsgs[i].teamid << ", ";
    //     // }
    //     // std::cout << std::endl;

    //     // teamIDs.erase(255); // Remove team id used by connectors (255)

    //     // int isClosestCount = 0;

    //     // /* Check whether itself is closest to a non team robot in every team  */
    //     // for (auto itr = teamIDs.begin(); itr != teamIDs.end(); ++itr) {

    //     //     UInt8 tempTeamID = *itr;

    //     //     /* Initializa a list of non team robots that this robot is the closest */
    //     //     std::vector<std::string> closestNonTeamRobots; // CURRENTLY CHECKS FOR ONLY ONE FOR OPTIMIZATION

    //     //     /* Extract the messages with the team ID considered in the current iteration */
    //     //     std::vector<Message> tempTeamMsgs;
    //     //     std::vector<Message> tempNonTeamMsgs;

    //     //     for(size_t i = 0; i < nonTeamMsgs.size(); i++) {
    //     //         if(nonTeamMsgs[i].teamid == tempTeamID)
    //     //             tempTeamMsgs.push_back(nonTeamMsgs[i]);
    //     //         else
    //     //             tempNonTeamMsgs.push_back(nonTeamMsgs[i]);
    //     //     }

    //     //     for(size_t i = 0; i < tempNonTeamMsgs.size(); i++) {

    //     //         std::cout << "Checking " << tempNonTeamMsgs[i].id << std::endl;

    //     //         /* Find the distance between itself and the non team robot */
    //     //         Real myDist = tempNonTeamMsgs[i].direction.Length();

    //     //         /* Flag to check whether there is a team robot closer than itself to a non team robot */
    //     //         bool isClosest = true;

    //     //         for(size_t j = 0; j < tempTeamMsgs.size(); j++) {
                    
    //     //             std::vector<std::string> connections = tempTeamMsgs[j].connections;

    //     //             std::cout << "(" << tempTeamMsgs[j].id << ") ";
    //     //             for(size_t k = 0; k < connections.size(); k++)
    //     //                 std::cout << connections[k] << ", ";
    //     //             std::cout << std::endl;

    //     //             // Check if the team robot has seen the non-team robot 
    //     //             if (std::find(connections.begin(), connections.end(), tempNonTeamMsgs[i].id) != connections.end()) {

    //     //                 /* Check the distance between the team robot and the non team robot*/
    //     //                 CVector2 diff = tempNonTeamMsgs[i].direction - tempTeamMsgs[j].direction;
    //     //                 Real dist = diff.Length();

    //     //                 if(dist < myDist) {
    //     //                     isClosest = false;  // Not the closest to the non team robot
    //     //                     break;
    //     //                 }
    //     //             }
    //     //         }
    //     //         if(isClosest) {
    //     //             closestNonTeamRobots.push_back(tempNonTeamMsgs[i].id);
    //     //             std::cout << "Closest to: " << tempNonTeamMsgs[i].id << std::endl;
    //     //             break;
    //     //         }
    //     //     }
    //     //     if( !closestNonTeamRobots.empty() )
    //     //         isClosestCount++;
    //     // }

    //     // /* Must be closest to at least one robot from each team */
    //     // if(isClosestCount == teamIDs.size())
    //     //     isClosestToNonTeam = true;
    //     // else
    //     //     isClosestToNonTeam = false;
        









    //     // /* Check whether a team is within its communication range */
    //     // for(size_t i = 0; i < nonTeamMsgs.size(); i++) {
    //     //     if(nonTeamMsgs[i].state != RobotState::CONNECTOR) {
    //     //         UInt8 team = nonTeamMsgs[i].teamid;
    //     //         hops[team].count = 1;
    //     //     }
    //     // }
        
    //     // for (auto& it: hops) {
    //     //     std::cout << "Team: " << it.first << ", Hop: " << it.second.count << std::endl;
    //     // }
    // }
}

/****************************************/
/****************************************/

CVector2 CFollower::GetTeamFlockingVector() {

    CVector2 resVec = CVector2();

    if(leaderMsg.direction.Length() > 0.0f) {

        resVec = leaderMsg.direction;

    } else {
        
        if(hopCountToLeader == 255)
            return CVector2();  // No attraction
        
        size_t numAttract = 0;

        /* Calculate attractive force towards team members with a smaller hop count */
        for(size_t i = 0; i < teamMsgs.size(); i++) {
            if(teamMsgs[i].hops[teamID].count < hopCountToLeader) {
                resVec += teamMsgs[i].direction;
                numAttract++;
            }
        }
        resVec /= numAttract;
    }

    /* Run the PID controller to calculate the force to team members with the smallest hop count */
    Real fPID = pid->calculate(m_sLeaderFlockingParams.TargetDistance,
                               resVec.Length());

    resVec = CVector2(-fPID,
                      resVec.Angle());

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CFollower::GetRobotRepulsionVector() {
    CVector2 resVec = CVector2();

    std::vector<Message> repulseMsgs;

    /* Add team messages with equal or greater hop count */
    if(hopCountToLeader < 255) {
        UInt8 minCount = hopCountToLeader - 1;

        for(size_t i = 0; i < teamMsgs.size(); i++) {
            if(teamMsgs[i].hops[teamID].count > minCount)
                repulseMsgs.push_back(teamMsgs[i]);
        }
    } else {
        repulseMsgs.insert(std::end(repulseMsgs), std::begin(teamMsgs), std::end(teamMsgs));
    }
    
    /* Add other messages */
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));

    for(size_t i = 0; i < repulseMsgs.size(); i++) {
        /* Calculate LJ */
        Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJonesRepulsion(repulseMsgs[i].direction.Length());
        /* Sum to accumulator */
        resVec += CVector2(fLJ,
                           repulseMsgs[i].direction.Angle());
    }

    /* Calculate the average vector */
    if( !repulseMsgs.empty() )
        resVec /= repulseMsgs.size();

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CFollower::GetObstacleRepulsionVector() {
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

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

void CFollower::Flock() {
    /* Calculate overall force applied to the robot */
    CVector2 teamForce     = GetTeamFlockingVector();
    CVector2 robotForce    = GetRobotRepulsionVector();
    CVector2 obstacleForce = GetObstacleRepulsionVector();
    CVector2 sumForce      = teamWeight*teamForce + teamWeight*robotForce + obstacleWeight*obstacleForce;

    /* DEBUGGING */
    if(this->GetId() == "F1") {
        std::cout << "team: " << teamForce.Length() << std::endl;
        std::cout << "robot: " << robotForce.Length() << std::endl;
        std::cout << "obstacle: " << obstacleForce.Length() << std::endl;
        std::cout << "sum: " << sumForce.Length() << std::endl;
    }

    /* Set Wheel Speed */
    // if(sumForce.Length() > 1.0f)
    //     SetWheelSpeedsFromVector(sumForce);
    // else
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

/****************************************/
/****************************************/

void CFollower::SetWheelSpeedsFromVector(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
    /* State transition logic */
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
        if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
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

void CFollower::PrintName() {
    std::cout << "[" << this->GetId() << "] ";
}

/****************************************/
/****************************************/

UInt8 CFollower::GetTeamID() {
    return teamID;
}

/****************************************/
/****************************************/

void CFollower::SetTeamID(const UInt8 id) {
    teamID = id;
    currentState = RobotState::FOLLOWER;
}

/****************************************/
/****************************************/

bool CFollower::IsWorking() {
    return performingTask;
}

/****************************************/
/****************************************/

/* Callback functions (Controllable events) */

// void CFollower::Callback_MoveFlock(void* data) {
//     std::cout << "Action: moveFlock" <<std::endl;
//     currentMoveType = MoveType::FLOCK;
// }

// void CFollower::Callback_MoveStop(void* data) {
//     std::cout << "Action: moveStop" <<std::endl;
//     currentMoveType = MoveType::STOP;
// }

// void CFollower::Callback_TaskBegin(void* data) {
//     std::cout << "Action: taskBegin" <<std::endl;
//     performingTask = true;
// }

// void CFollower::Callback_TaskStop(void* data) {
//     std::cout << "Action: taskStop" <<std::endl;
//     performingTask = false;
// }

// void CFollower::Callback_SetFS(void* data) {
//     std::cout << "Action: setFS" <<std::endl;
//     currentState = RobotState::FOLLOWER;

//     /* Join the closest team */
//     std::vector<Message> msgs(otherTeamMsgs);
//     msgs.insert(std::end(msgs),
//                 std::begin(otherLeaderMsgs),
//                 std::end(otherLeaderMsgs));

//     Real closestTeamDistance = __FLT_MAX__;
//     UInt8 closestTeamID;

//     for(int i = 0 ; i < msgs.size(); i++) {
//         Real dist = msgs[i].direction.Length();
//         if(dist < closestTeamDistance) {
//             closestTeamDistance = dist;
//             closestTeamID = msgs[i].teamid;
//         }
//     }

//     teamID = closestTeamID;
// }

// void CFollower::Callback_SetCS(void* data) {
//     std::cout << "Action: setCS" <<std::endl;
//     currentState = RobotState::CONNECTOR;
//     teamID = 255;
// }

void CFollower::Callback_MoveFlock(void* data) {
    std::cout << "Action: moveFlock" <<std::endl;
    currentMoveType = MoveType::FLOCK;
}

void CFollower::Callback_MoveStop(void* data) {
    std::cout << "Action: moveStop" <<std::endl;
    currentMoveType = MoveType::STOP;
}

void CFollower::Callback_SetF(void* data) {
    std::cout << "Action: setF" <<std::endl;
}

void CFollower::Callback_SetC(void* data) {
    std::cout << "Action: setC" <<std::endl;
}

void CFollower::Callback_SendR(void* data) {
    std::cout << "Action: sendR" <<std::endl;

    // Send request 
}

void CFollower::Callback_SendA(void* data) {
    std::cout << "Action: sendA" <<std::endl;
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

// unsigned char CFollower::Check_ReceiveTB(void* data) {
//     if(leaderMsg.direction.Length() > 0.0f && leaderMsg.contents[0] == "1") {
//         std::cout << "Event: " << 1 << " - receiveTB" << std::endl;
//         return 1;
//     }
//     std::cout << "Event: " << 0 << " - receiveTB" << std::endl;
//     return 0;
// }

// unsigned char CFollower::Check_ReceiveTS(void* data) {
//     if(leaderMsg.direction.Length() > 0.0f && leaderMsg.contents[0] == "0") {
//         std::cout << "Event: " << 1 << " - receiveTS" << std::endl;
//         return 1;
//     }
//     std::cout << "Event: " << 0 << " - receiveTS" << std::endl;
//     return 0;
// }

// unsigned char CFollower::Check_DistFar(void* data) {
//     if(minNonTeamDistance != __FLT_MAX__ && minNonTeamDistance >= separationThres) {
//         std::cout << "Event: " << 1 << " - distFar" << std::endl;
//         return 1;
//     }
//     std::cout << "Event: " << 0 << " - distFar" << std::endl;
//     return 0;
// }

// unsigned char CFollower::Check_DistNear(void* data) {
//     if(minNonTeamDistance != __FLT_MAX__ && minNonTeamDistance < joiningThres) {
//         std::cout << "Event: " << 1 << " - distNear" << std::endl;
//         return 1;
//     }
//     std::cout << "Event: " << 0 << " - distNear" << std::endl;
//     return 0;
// }

// unsigned char CFollower::Check_IsNearest(void* data) {
//     if(isClosestToNonTeam) {
//         std::cout << "Event: " << 1 << " - isNearest" << std::endl;
//         return 1;
//     }
//     std::cout << "Event: " << 0 << " - isNearest" << std::endl;
//     return 0;
// }

// unsigned char CFollower::Check_NotNearest(void* data) {
//     if(!isClosestToNonTeam) {
//         std::cout << "Event: " << 1 << " - notNearest" << std::endl;
//         return 1;
//     }
//     std::cout << "Event: " << 0 << " - notNearest" << std::endl;
//     return 0;
// }

// unsigned char CFollower::Check_CondF1(void* data) {

// }

// unsigned char CFollower::Check_NotCondF1(void* data) {

// }

// unsigned char CFollower::Check_CondF2(void* data) {

// }

// unsigned char CFollower::Check_NotCondF2(void* data) {

// }

unsigned char CFollower::Check_CondC1(void* data) {
    if(connectionCandidate.direction.Length() >= separationThres) {
        std::cout << "Event: " << 1 << " - condC1" << std::endl;
        return 1;
    }
    std::cout << "Event: " << 0 << " - condC1" << std::endl;
    return 0;
}

unsigned char CFollower::Check_NotCondC1(void* data) {
    if(connectionCandidate.direction.Length() >= separationThres) {
        std::cout << "Event: " << 0 << " - notCondC1" << std::endl;
        return 0;
    }
    std::cout << "Event: " << 1 << " - notCondC1" << std::endl;
    return 1;
}

unsigned char CFollower::Check_CondC2(void* data) {
    std::cout << "Event: " << 0 << " - condC2" << std::endl;
    return 0;
}

unsigned char CFollower::Check_NotCondC2(void* data) {
    std::cout << "Event: " << 1 << " - notCondC2" << std::endl;
    return 1;
}

unsigned char CFollower::Check_CondC3(void* data) {
    if(connectionCandidate.direction.Length() > 0.0f) {
        if(teamID < connectionCandidate.teamID) {
            std::cout << "Event: " << 1 << " - condC3" << std::endl;
            return 1;
        }
    }
    std::cout << "Event: " << 0 << " - condC3" << std::endl;
    return 0;
}

unsigned char CFollower::Check_NotCondC3(void* data) {
    if(connectionCandidate.direction.Length() > 0.0f) {
        if(teamID < connectionCandidate.teamID) {
            std::cout << "Event: " << 0 << " - notCondC3" << std::endl;
            return 0;
        }
    }
    std::cout << "Event: " << 1 << " - notCondC3" << std::endl;
    return 1;
}

unsigned char CFollower::Check_ReceiveR(void* data) {

}

unsigned char CFollower::Check_ReceiveA(void* data) {

}

unsigned char CFollower::Check_ReceiveNA(void* data) {

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
REGISTER_CONTROLLER(CFollower, "follower_controller")
