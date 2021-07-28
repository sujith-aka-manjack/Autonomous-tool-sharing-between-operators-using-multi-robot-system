/* Include the controller definition */
#include "follower.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <utility/team_color.h>
#include <algorithm>
#include <unordered_set>

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

/* 
* Checks whethe the Message is empty or not by checking the direction it was received from
*/
bool CFollower::Message::Empty() {
    return direction.Length() == 0.0f;
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

        /* Timeout duration */
        GetNodeAttribute(GetNode(t_node, "timeout"), "request", requestDuration);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Initialization */
    currentState = RobotState::CONNECTOR; // Set initial state to connector
    requestTimer = 0;
    performingTask = false; // Robot initially not working on any task
    hopCountToLeader = 255; // Default (max) value as hop count is unknown
    shareToLeader = "";
    shareToTeam = "";
    initStepComplete = 0;

    /*
    * Init SCT Controller
    */
    sct = new SCT();

    /* Register controllable events */
    sct->add_callback(this, EV_moveFlock, &CFollower::Callback_MoveFlock, NULL, NULL);
    sct->add_callback(this, EV_moveStop,  &CFollower::Callback_MoveStop,  NULL, NULL);
    sct->add_callback(this, EV_setF,      &CFollower::Callback_SetF,      NULL, NULL);
    sct->add_callback(this, EV_setC,      &CFollower::Callback_SetC,      NULL, NULL);
    sct->add_callback(this, EV_sendRL,    &CFollower::Callback_SendRL,    NULL, NULL);
    sct->add_callback(this, EV_sendRC,    &CFollower::Callback_SendRC,    NULL, NULL);
    sct->add_callback(this, EV_sendA,     &CFollower::Callback_SendA,     NULL, NULL);
    
    /* Register uncontrollable events */
    sct->add_callback(this, EV_condC1,    NULL, &CFollower::Check_CondC1,    NULL);
    sct->add_callback(this, EV_notCondC1, NULL, &CFollower::Check_NotCondC1, NULL);
    sct->add_callback(this, EV_condC2,    NULL, &CFollower::Check_CondC2,    NULL);
    sct->add_callback(this, EV_notCondC2, NULL, &CFollower::Check_NotCondC2, NULL);
    sct->add_callback(this, EV_condC3,    NULL, &CFollower::Check_CondC3,    NULL);
    sct->add_callback(this, EV_notCondC3, NULL, &CFollower::Check_NotCondC3, NULL);
    sct->add_callback(this, EV_nearC,     NULL, &CFollower::Check_NearC,     NULL);
    sct->add_callback(this, EV_notNearC,  NULL, &CFollower::Check_NotNearC,  NULL);
    sct->add_callback(this, EV_condF1,    NULL, &CFollower::Check_CondF1,    NULL);
    sct->add_callback(this, EV_notCondF1, NULL, &CFollower::Check_NotCondF1, NULL);
    sct->add_callback(this, EV_condF2,    NULL, &CFollower::Check_CondF2,    NULL);
    sct->add_callback(this, EV_notCondF2, NULL, &CFollower::Check_NotCondF2, NULL);
    sct->add_callback(this, EV_receiveA,  NULL, &CFollower::Check_ReceiveA,  NULL);
    sct->add_callback(this, EV_receiveNA, NULL, &CFollower::Check_ReceiveNA, NULL);
    sct->add_callback(this, EV_receiveR,  NULL, &CFollower::Check_ReceiveR,  NULL);

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
    msg = CByteArray(91, 255);
    m_pcRABAct->SetData(msg);
    msg_index = 0;

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

const std::map<UInt8, CFollower::HopMsg>& CFollower::GetHops() const {
    return hops;
}

/****************************************/
/****************************************/

bool CFollower::IsWorking() {
    return performingTask;
}

/****************************************/
/****************************************/

void CFollower::ControlStep() {

    std::cout << "\n---------- " << this->GetId() << " ----------" << std::endl;

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/

    /* Create new msg */
    msg = CByteArray(91, 255);
    msg_index = 0;

    /* Clear messages received */
    leaderMsg = Message();
    teamMsgs.clear();
    connectorMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();

    cmsgToSend.clear();

    leaderSignal = 255; // Default value for no signal
    hopCountToLeader = 255; // Default value for not known hop count to the leader

    /* Reset sensor reading results */
    condC2 = false;
    receiveR  = false;
    receiveA  = false;
    receiveNA = false;

    robotsToAccept.clear();

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();

    /*------------------------*/
    /* Update sensor readings */
    /*------------------------*/
    Update();

    /*--------------------*/
    /* Run SCT controller */
    /*--------------------*/
    std::cout << "--- Supervisors ---" << std::endl;

    if(initStepComplete > 0)
        sct->run_step();    // Run the supervisor to get the next action
    else
        initStepComplete++;

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

            /* Hop count */
            /* Set its hop count to the leader */
            std::cout << "Hops to leader: " << hopCountToLeader << std::endl;
            msg[msg_index++] = 1; // Number of HopMsg

            msg[msg_index++] = teamID;
            msg[msg_index++] = hopCountToLeader;
            msg_index += 2; // Skip ID

            msg_index += 4; // Skip to next part

            break;
        }
        case RobotState::CONNECTOR: {
            std::cout << "State: CONNECTOR" << std::endl;
            m_pcLEDs->SetAllColors(CColor::CYAN);

            /* Leader signal */
            msg_index++; // Skip to next part

            /* Hop count */
            msg[msg_index++] = hops.size(); // Set the number of HopMsg

            for(const auto& it : hops) {

                msg[msg_index++] = it.first;                     // Team ID
                msg[msg_index++] = it.second.count;              // Count

                if( it.second.ID.empty() )
                    msg_index += 2; // Skip
                else {
                    msg[msg_index++] = it.second.ID[0];              // ID
                    msg[msg_index++] = stoi(it.second.ID.substr(1)); // ID
                }
            }
            // Skip if not all bytes are used
            msg_index += (2 - hops.size()) * 4; // TEMP: Currently assuming only two teams

            break;
        }
        case RobotState::LEADER: {
            std::cout << "State: LEADER. Something went wrong." << std::endl;
            break;
        }
    }

    /* Movement */
    switch(currentMoveType) {
        case MoveType::FLOCK: {
            Flock();
            break;
        }
        case MoveType::STOP: {
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            break;
        }
    }

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
    if( !shareToLeader.empty() ) {
        msg[msg_index++] = shareToLeader[0];
        msg[msg_index++] = stoi(shareToLeader.substr(1));
    } else
        msg_index += 2;

    std::cout << "Share to leader: " << shareToLeader << std::endl;

    if( !shareToTeam.empty() ) {
        msg[msg_index++] = shareToTeam[0];
        msg[msg_index++] = stoi(shareToTeam.substr(1));
    } else
        msg_index += 2;

    std::cout << "Share to team: " << shareToTeam << std::endl;

    /* Set ID of all connections to msg */
    std::vector<Message> allMsgs(teamMsgs);
    allMsgs.insert(std::end(allMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

    if( !leaderMsg.Empty() ) {
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

            // std::cout << tMsgs[i].Data << std::endl;

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

                // std::cout << "FROM: " << conMsg.from << std::endl;

                robotID = "";
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                conMsg.to = robotID;
                
                // std::cout << "TO: " << conMsg.to << std::endl;
                
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
            
            if(tMsgs[i].Data[index] == 255) {
                index += 2;
            } else {
                robotID = "";
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                msg.shareToTeam = robotID;
            }

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

void CFollower::Update() {

    std::cout << "leaderMsg = " << ( !leaderMsg.Empty() ) << std::endl;
    std::cout << "teamMsg = " << teamMsgs.size() << std::endl;
    std::cout << "otherLMsg = " << otherLeaderMsgs.size() << std::endl;
    std::cout << "otherTMsg = " << otherTeamMsgs.size() << std::endl;
    std::cout << "connectorMsg  = " << connectorMsgs.size() << std::endl;

    if(currentState == RobotState::FOLLOWER) {

        GetLeaderInfo();
        SetConnectorToRelay();

        connectionCandidate = GetClosestNonTeam();

        if( !connectionCandidate.Empty() )
            condC2 = IsClosestToRobot(connectionCandidate);

        /* Check whether it has received an accept message */
        if(currentRequest.type == 'R') {
            CheckAccept();

            /* Decrement timer */
            requestTimer--;
            std::cout << "requestTimer: " << requestTimer << std::endl;

            /* Check whether an Accept message was not received before the timeout */
            if(requestTimer == 0 && currentAccept.type == 'N') {
                receiveNA = true;
                currentRequest = ConnectionMsg(); // Clear current request
            }
        }
            
            // Remember currently sending request? (with timesteps to continue sending?)

        SetCMsgsToRelay();
        
    } else if(currentState == RobotState::CONNECTOR) {

        // TODO: Check if connections with connectors are all present
            // Extract ID in hops to a vector (no duplicate)
            // Loop connectorMsgs
                // Delete seen connector IDs
            // If no duplicate vector is empty, all connections with connectors remain
        
        // DEBUG
        // std::cout << "--- HOPS ---" << std::endl;
        // for(const auto& it : hops) {
        //     std::cout << "Team: " << it.first << std::endl;
        //     std::cout << "Count: " << it.second.count << std::endl;
        //     std::cout << "ID: " << it.second.ID << std::endl;
        // }

        CheckRequests();

        UpdateHopCounts();
    }
}

/****************************************/
/****************************************/

void CFollower::GetLeaderInfo() {

    /* Find the hop count to and signal from the leader */
    if( !leaderMsg.Empty() ) { // Leader is in range

        hopCountToLeader = 1;
        leaderSignal = leaderMsg.leaderSignal;

    } else { // Leader is not in range. Relay leader signal

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
}

/****************************************/
/****************************************/

CFollower::Message CFollower::GetClosestNonTeam() {
    
    /* Check for the robot that this robot can connect */
    Real minDist = 255;
    std::vector<Message> candidateMsgs;
    Message closestRobot;

    /* Prioritize connectors over other team members */
    if( !connectorMsgs.empty() ) {
        candidateMsgs = connectorMsgs;            
    } else {
        candidateMsgs = otherLeaderMsgs;
        candidateMsgs.insert(std::end(candidateMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
    }

    /* Find the closest non team robot */
    for(size_t i = 0; i < candidateMsgs.size(); i++) {
        Real dist = candidateMsgs[i].direction.Length();
        if(dist < minDist) {

            /* Check robot is either a follower or in the case of a connector, it has a hop count = 1 to its current team */
            if(candidateMsgs[i].state == RobotState::FOLLOWER || 
               (candidateMsgs[i].state == RobotState::CONNECTOR && candidateMsgs[i].hops[teamID].count == 1)) {

                minDist = dist;
                closestRobot = candidateMsgs[i];
            }
        }
    }

    if(minDist < 255)
        std::cout << "Dist to candidate: " << minDist << std::endl;
    
    return closestRobot;
}

/****************************************/
/****************************************/

bool CFollower::IsClosestToRobot(const Message& msg) {
    
    Real myDist = msg.direction.Length();

    /* If the team has identified the next connector to connect to, check if it is the same */
    if( !shareToTeam.empty() ) {
        if(msg.ID != shareToTeam)
            return false;
    }

    /* Check whether it is the closest to the candidate among other followers in the team that sees it (condC2) */
    for(size_t i = 0; i < teamMsgs.size(); i++) {

        std::vector<std::string> connections = teamMsgs[i].connections;

        // std::cout << teamMsgs[i].ID << ": ";
        // for(size_t j = 0; j < teamMsgs[i].connections.size(); j++)
        //     std::cout << teamMsgs[i].connections[j] << ", ";
        // std::cout << std::endl;

        // Check if the team robot has seen the non-team robot 
        if (std::find(connections.begin(), connections.end(), msg.ID) != connections.end()) {

            /* Check the distance between its candidate and nearby team robots */
            CVector2 diff = msg.direction - teamMsgs[i].direction;
            Real dist = diff.Length();

            if(dist < myDist)
                return false;  // Not the closest to the candidate robot
        }
    }
    return true;
}

/****************************************/
/****************************************/

void CFollower::CheckAccept() {

    /* Request sent to leader */
    if(currentRequest.to[0] == 'L') {

        std::vector<Message> combinedTeamMsgs(teamMsgs);
        if( !leaderMsg.Empty() )
            combinedTeamMsgs.push_back(leaderMsg);

        for(const auto& teamMsg : combinedTeamMsgs) {
            for(const auto& cmsg : teamMsg.cmsg) {
                if(cmsg.type == 'A'){

                    if(cmsg.to == this->GetId()) {    // Request approved for this follower
                        receiveA = true;
                        currentAccept = cmsg;
                    } else                            // Request approved for another follower
                        receiveNA = true;
                }
            }
        }
    } 
    /* Request sent to connector */
    else {
        for(const auto& msg : connectorMsgs) {
            for(const auto& cmsg : msg.cmsg) {
                if(cmsg.type == 'A') {  // TEMP: Connector always sends accept messages

                    /* Check the connector matches its original request and is directed to its current team */
                    if(cmsg.from == currentRequest.to && cmsg.toTeam == teamID) {

                        if(cmsg.to == this->GetId()) {    // Request approved for this follower
                            receiveA = true;
                            currentAccept = cmsg;
                            hopsCopy = msg.hops;
                        } else                            // Request approved for another follower
                            receiveNA = true;
                    }
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CFollower::CheckRequests() {

    /* Check all requests sent to itself and choose one to respond to each team */
    for(const auto& msg : otherTeamMsgs) {
        for(const auto& cmsg : msg.cmsg) {
            if(cmsg.to == this->GetId() && cmsg.type == 'R') {

                receiveR = true;

                if(hops[msg.teamID].count == 1) {   // Only accept if it does not have a fixed connector (count = 1)

                    /* Accept first request seen for a team */
                    if(robotsToAccept.find(msg.teamID) == robotsToAccept.end()) {
                        robotsToAccept[msg.teamID] = cmsg.from;
                        continue;
                    }

                    UInt8 currentFID = stoi(robotsToAccept[msg.teamID].substr(1));
                    UInt8 newFID = stoi(cmsg.from.substr(1));

                    /* Send an accept message to the follower with the smallest ID */
                    if(newFID < currentFID)
                        robotsToAccept[msg.teamID] = cmsg.from;
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CFollower::SetCMsgsToRelay() {

    /* Combine messages from the leader and other followers that belong in the same team */
    std::vector<Message> combinedTeamMsgs(teamMsgs);
    combinedTeamMsgs.push_back(leaderMsg);

    /* Booleans to only relay up to 1 request and accept messages each */
    bool receivedRequest = false;
    bool receivedAccept = false;

    for(const auto& msg : combinedTeamMsgs) {
        for(const auto& cmsg : msg.cmsg) {
            auto hops = msg.hops;

            /* Relay Request message received from robots with greater hop count */
            if( !receivedRequest ) {
                if(cmsg.type == 'R' && hops[teamID].count > hopCountToLeader) {
                    if(currentRequest.type != 'R') { // Check if it is currently not requesting
                        cmsgToSend.push_back(cmsg);
                        receivedRequest = true;
                        std::cout << "Relay Request, from: " << cmsg.from << " to: " << cmsg.to << std::endl;
                    }
                }
            }
            
            /* Relay Accept message received from robots with smaller hop count */
            if( !receivedAccept ) {
                if(cmsg.type == 'A' && hops[teamID].count < hopCountToLeader) {
                    cmsgToSend.push_back(cmsg);
                    receivedAccept = true;
                    std::cout << "Relay Accept, from: " << cmsg.from << " to: " << cmsg.to << std::endl;
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CFollower::SetConnectorToRelay() {
 
    /* Check if a connector with a hop count = 1 to its current team is nearby */
    bool foundFirstConnector = false;
    for(const auto& msg : connectorMsgs) {
        auto hopInfo = msg.hops;

        if(hopInfo[teamID].count == 1) {

            /* Send info of connector found */
            shareToLeader = msg.ID;
            foundFirstConnector = true;
            break;
        }
    }

    /* Combine messages from the leader and other followers that belong in the same team */
    std::vector<Message> combinedTeamMsgs(teamMsgs);
    combinedTeamMsgs.push_back(leaderMsg);

    /* If the first connector is not nearby, check which it should relay upstream */
    if( !foundFirstConnector ) {
        if( !combinedTeamMsgs.empty() ) {

            bool previousSeen = false;
            bool newValue = false;

            for(const auto& msg : combinedTeamMsgs) {
                auto hopInfo = msg.hops;

                if(hopInfo[teamID].count > hopCountToLeader) {
                    if(msg.shareToLeader == shareToLeader)
                        previousSeen = true; // If only same info, send the same connector
                    else if( !msg.shareToLeader.empty() ) {

                        /* Update to connector info that's different from previous */
                        shareToLeader = msg.shareToLeader;
                        newValue = true;
                        break;
                    }
                }
            }

            if( !previousSeen && !newValue ) // If previous info not received and no new info, send nothing
                shareToLeader = "";
            
        } else // If no upstream exists, send nothing
            shareToLeader = "";
    }

    /* Get connector info to relay downstream */
    if( !combinedTeamMsgs.empty() ) {
        for(const auto& msg : combinedTeamMsgs) {
            auto hopInfo = msg.hops;

            /* Relay the first seen connector info from leader O(1) */
            if(hopInfo[teamID].count < hopCountToLeader) {
                shareToTeam = msg.shareToTeam;
                break;
            }
        }
    } else // If no downstream exists, send nothing
        shareToTeam = "";
}

/****************************************/
/****************************************/

void CFollower::UpdateHopCounts() {

    /* Extract connector IDs to check */
    std::unordered_set<std::string> robotIDs;

    for(const auto& hop : hops) {
        if( !hop.second.ID.empty() )
            robotIDs.insert(hop.second.ID);
    }

    /* Extract Messages from connectors that have the IDs found previously */
    std::unordered_map<std::string, Message> robotMessages;

    for(const auto& msg : connectorMsgs) {

        if(robotIDs.empty())
            break;

        for(auto& id : robotIDs) {
            if(msg.ID == id) {
                robotMessages[id] = msg;
                robotIDs.erase(id);
                break;
            }
        }
    }

    /* Update hop count */
    for(auto& hop : hops) {
        std::string previousRobotID = hop.second.ID;

        if( !previousRobotID.empty() ) {
            UInt8 teamToCheck = hop.first;
            HopMsg previousHop = robotMessages[previousRobotID].hops[teamToCheck];
            hop.second.count = previousHop.count + 1; // Increment by 1
        }
    }
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
    if(sumForce.Length() > 1.0f)
        SetWheelSpeedsFromVector(sumForce);
    else
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

/****************************************/
/****************************************/

CVector2 CFollower::GetTeamFlockingVector() {

    CVector2 resVec = CVector2();

    if( !leaderMsg.Empty() ) {

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

/* Callback functions (Controllable events) */

// void CFollower::Callback_TaskBegin(void* data) {
//     std::cout << "Action: taskBegin" <<std::endl;
//     performingTask = true;
// }

// void CFollower::Callback_TaskStop(void* data) {
//     std::cout << "Action: taskStop" <<std::endl;
//     performingTask = false;
// }

void CFollower::Callback_MoveFlock(void* data) {
    std::cout << "Action: moveFlock" <<std::endl;
    currentMoveType = MoveType::FLOCK;
    currentRequest = ConnectionMsg(); // Clear any existing requests
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
    
    if(currentAccept.from[0] == 'L') {  // Accept received from the leader

        /* Add every other visible team to hop map */
        for(const auto& msg : otherTeamMsgs) {

            /* Add hop count entry if not yet registered */
            if(hops.find(msg.teamID) == hops.end()) {
                HopMsg hop;
                hop.count = 1;
                hops[msg.teamID] = hop;
            }
        }
    } else {    // Accept received from a connector

        /* Use the connector to generate its hop count to other teams */
        hopsCopy.erase(teamID);            // Delete entry of its own team

        for(auto& it : hopsCopy) {         // Loop to add hop count of 1 to each item
            it.second.count++;
            it.second.ID = currentAccept.from;
        }
        hops = hopsCopy;                   // Set to its hops
    }

    /* Set hop count to the team it is leaving to 1 */
    HopMsg hop;
    hop.count = 1;
    hops[teamID] = hop;

    /* Reset variables */
    currentAccept = ConnectionMsg(); 
    hopsCopy.clear();
    shareToLeader = "";
    shareToTeam = "";

    currentState = RobotState::CONNECTOR;
    teamID = 255;
}

void CFollower::Callback_SendRL(void* data) {
    std::cout << "Action: sendRL" <<std::endl;

    /* Set request to send */
    ConnectionMsg cmsg;
    cmsg.type   = 'R';
    cmsg.from   = this->GetId();
    cmsg.to     = "L" + std::to_string(teamID);
    cmsg.toTeam = teamID;
    cmsgToSend.push_back(cmsg);

    currentRequest = cmsg;
    requestTimer = requestDuration;
    std::cout << "requestTimer SET: " << requestTimer << std::endl;
}

void CFollower::Callback_SendRC(void* data) {
    std::cout << "Action: sendRC" <<std::endl;

    /* Set request to send */
    ConnectionMsg cmsg;
    cmsg.type   = 'R';
    cmsg.from   = this->GetId();
    cmsg.to     = connectionCandidate.ID;
    cmsg.toTeam = 255; // No team
    cmsgToSend.push_back(cmsg);

    currentRequest = cmsg;
    requestTimer = requestDuration;
    std::cout << "requestTimer SET: " << requestTimer << std::endl;
}

void CFollower::Callback_SendA(void* data) {
    std::cout << "Action: sendA" <<std::endl;

    for(const auto& it : robotsToAccept) {
        ConnectionMsg cmsg;
        cmsg.type   = 'A';
        cmsg.from   = this->GetId();
        cmsg.to     = it.second;
        cmsg.toTeam = it.first;
        cmsgToSend.push_back(cmsg);

        /* Update hop count to the team using the new connector */
        hops[it.first].count++; // 1 -> 2
        hops[it.first].ID = it.second;
    }
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
    std::cout << "Event: " << condC2 << " - condC2" << std::endl;
    return condC2;
}

unsigned char CFollower::Check_NotCondC2(void* data) {
    std::cout << "Event: " << !condC2 << " - notCondC2" << std::endl;
    return !condC2;
}

unsigned char CFollower::Check_CondC3(void* data) {
    if( !connectionCandidate.Empty()) {
        if(teamID < connectionCandidate.teamID) {
            std::cout << "Event: " << 1 << " - condC3" << std::endl;
            return 1;
        }
    }
    std::cout << "Event: " << 0 << " - condC3" << std::endl;
    return 0;
}

unsigned char CFollower::Check_NotCondC3(void* data) {
    if( !connectionCandidate.Empty() ) {
        if(teamID < connectionCandidate.teamID) {
            std::cout << "Event: " << 0 << " - notCondC3" << std::endl;
            return 0;
        }
    }
    std::cout << "Event: " << 1 << " - notCondC3" << std::endl;
    return 1;
}

unsigned char CFollower::Check_NearC(void* data) {
    bool connectorSeen = !connectorMsgs.empty();
    std::cout << "Event: " << connectorSeen << " - nearC" << std::endl;
    return connectorSeen;
}

unsigned char CFollower::Check_NotNearC(void* data) {
    bool connectorSeen = !connectorMsgs.empty();
    std::cout << "Event: " << !connectorSeen << " - notNearC" << std::endl;
    return !connectorSeen;
}

unsigned char CFollower::Check_CondF1(void* data) {
    return 0;
}

unsigned char CFollower::Check_NotCondF1(void* data) {
    return 1;
}

unsigned char CFollower::Check_CondF2(void* data) {
    return 0;
}

unsigned char CFollower::Check_NotCondF2(void* data) {
    return 1;
}

unsigned char CFollower::Check_ReceiveR(void* data) {
    std::cout << "Event: " << receiveR << " - receiveR" << std::endl;
    return receiveR;
}

unsigned char CFollower::Check_ReceiveA(void* data) {
    std::cout << "Event: " << receiveA << " - receiveA" << std::endl;
    return receiveA;
}

unsigned char CFollower::Check_ReceiveNA(void* data) {
    std::cout << "Event: " << receiveNA << " - receiveNA" << std::endl;
    return receiveNA;
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
