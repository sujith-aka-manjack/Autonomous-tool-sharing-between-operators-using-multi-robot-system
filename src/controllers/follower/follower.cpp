/* Include the controller definition */
#include "follower.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
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

// Find whether two lines intersect
// Code modified from https://www.tutorialspoint.com/Check-if-two-line-segments-intersect
bool onLine(CVector2 start, CVector2 end, CVector2 point/* line l1, Point p */) {   //check whether p is on the line or not
   if(point.GetX() <= std::max(start.GetX(), end.GetX()) && point.GetX() <= std::min(start.GetX(), end.GetX()) &&
      (point.GetY() <= std::max(start.GetY(), end.GetY()) && point.GetY() <= std::min(start.GetY(), end.GetY())))
      return true;
   
   return false;
}

int direction(CVector2 a, CVector2 b, CVector2 c) {
   int val = (b.GetY()-a.GetY())*(c.GetX()-b.GetX())-(b.GetX()-a.GetX())*(c.GetY()-b.GetY());
   if (val == 0)
      return 0;     //colinear
   else if(val < 0)
      return 2;    //anti-clockwise direction
      return 1;    //clockwise direction
}

bool isIntersect(CVector2 move, CVector2 start, CVector2 end) {
   //four direction for two lines and points of other line
   int dir1 = direction(CVector2(), move, start);
   int dir2 = direction(CVector2(), move, end);
   int dir3 = direction(start, end, CVector2());
   int dir4 = direction(start, end, move);
   
   if(dir1 != dir2 && dir3 != dir4)
      return true; //they are intersecting

   if(dir1==0 && onLine(CVector2(), move, start)) //when p2 of line2 are on the line1
      return true;

   if(dir2==0 && onLine(CVector2(), move, end)) //when p1 of line2 are on the line1
      return true;

   if(dir3==0 && onLine(start, end, CVector2())) //when p2 of line1 are on the line2
      return true;

   if(dir4==0 && onLine(start, end, move)) //when p1 of line1 are on the line2
      return true;
         
   return false;
}

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

// void CFollower::SLeaderInteractionParams::Init(TConfigurationNode& t_node) {
//    try {
//       GetNodeAttribute(t_node, "target_distance", TargetDistance);
//       GetNodeAttribute(t_node, "kp", Kp);
//       GetNodeAttribute(t_node, "ki", Ki);
//       GetNodeAttribute(t_node, "kd", Kd);
//    }
//    catch(CARGoSException& ex) {
//       THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
//    }
// }

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
    sct(NULL) {}

/****************************************/
/****************************************/

CFollower::~CFollower() {
    delete sct;
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
        // m_sLeaderFlockingParams.Init(GetNode(t_node, "leader_flocking"));
        m_sTeamFlockingParams.Init(GetNode(t_node, "team_flocking"));

        /* Chain formation threshold */
        GetNodeAttribute(GetNode(t_node, "team_distance"), "separation_threshold", separationThres);
        GetNodeAttribute(GetNode(t_node, "team_distance"), "joining_threshold", joiningThres);

        /* Weights for the flocking behavior */
        GetNodeAttribute(GetNode(t_node, "flocking_weights"), "team",     teamWeight);
        GetNodeAttribute(GetNode(t_node, "flocking_weights"), "robot",    robotWeight);
        GetNodeAttribute(GetNode(t_node, "flocking_weights"), "obstacle", obstacleWeight);

        /* Timeout duration */
        GetNodeAttribute(GetNode(t_node, "timeout"), "send_message", sendDuration);
        GetNodeAttribute(GetNode(t_node, "timeout"), "wait_request", waitRequestDuration);

        /* SCT Model */
        GetNodeAttribute(GetNode(t_node, "SCT"), "path", m_strSCTPath);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
    std::cout << m_strSCTPath << std::endl;

    /* Initialization */
    currentState = RobotState::FOLLOWER; // Set initial state to connector
    requestTimer = 0;
    performingTask = false; // Robot initially not working on any task
    hopCountToLeader = 255; // Default (max) value as hop count is unknown
    shareToLeader = "";
    shareToTeam = "";
    shareDist = 255;
    setCTriggered = false; // Initialize flag
    initStepTimer = 0;

    /*
    * Init SCT Controller
    */
    sct = new SCT(m_strSCTPath);

    if( m_strSCTPath == "src/SCT_models/follower.yaml" ) {

        /* Without exchange */

        /* Register controllable events */
        sct->add_callback(this, sct->events["EV_moveFlock"], &CFollower::Callback_MoveFlock, NULL, NULL);
        sct->add_callback(this, sct->events["EV_moveStop"],  &CFollower::Callback_MoveStop,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_taskStart"], &CFollower::Callback_TaskStart, NULL, NULL);
        sct->add_callback(this, sct->events["EV_taskStop"],  &CFollower::Callback_TaskStop,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_switchF"],   &CFollower::Callback_SwitchF,   NULL, NULL);
        sct->add_callback(this, sct->events["EV_switchC"],   &CFollower::Callback_SwitchC,   NULL, NULL);
        sct->add_callback(this, sct->events["EV_requestL"],  &CFollower::Callback_RequestL,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_requestC"],  &CFollower::Callback_RequestC,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_respond"],   &CFollower::Callback_Respond,   NULL, NULL);
        // sct->add_callback(this, sct->events["EV_relay"],     &CFollower::Callback_Relay,     NULL, NULL);
        
        /* Register uncontrollable events */
        sct->add_callback(this, sct->events["EV_condC1"],    NULL, &CFollower::Check_CondC1,    NULL);
        sct->add_callback(this, sct->events["EV_notCondC1"], NULL, &CFollower::Check_NotCondC1, NULL);
        sct->add_callback(this, sct->events["EV_condC2"],    NULL, &CFollower::Check_CondC2,    NULL);
        sct->add_callback(this, sct->events["EV_notCondC2"], NULL, &CFollower::Check_NotCondC2, NULL);
        sct->add_callback(this, sct->events["EV_nearC"],     NULL, &CFollower::Check_NearC,     NULL);
        sct->add_callback(this, sct->events["EV_notNearC"],  NULL, &CFollower::Check_NotNearC,  NULL);
        sct->add_callback(this, sct->events["EV_condF1"],    NULL, &CFollower::Check_CondF1,    NULL);
        sct->add_callback(this, sct->events["EV_notCondF1"], NULL, &CFollower::Check_NotCondF1, NULL);
        sct->add_callback(this, sct->events["EV_condF2"],    NULL, &CFollower::Check_CondF2,    NULL);
        sct->add_callback(this, sct->events["EV_notCondF2"], NULL, &CFollower::Check_NotCondF2, NULL);
        sct->add_callback(this, sct->events["EV__respond"],  NULL, &CFollower::Check__Respond,  NULL);
        sct->add_callback(this, sct->events["EV_accept"],    NULL, &CFollower::Check_Accept,    NULL);
        sct->add_callback(this, sct->events["EV_reject"],    NULL, &CFollower::Check_Reject,    NULL);
        sct->add_callback(this, sct->events["EV__requestC"], NULL, &CFollower::Check__RequestC, NULL);
        sct->add_callback(this, sct->events["EV__start"],    NULL, &CFollower::Check__Start,    NULL);
        sct->add_callback(this, sct->events["EV__stop"],     NULL, &CFollower::Check__Stop,     NULL);
        // sct->add_callback(this, sct->events["EV__message"],  NULL, &CFollower::Check__Message,  NULL);
        // sct->add_callback(this, sct->events["EV__relay"],    NULL, &CFollower::Check__Relay,    NULL);

    } else if( m_strSCTPath == "src/SCT_models/follower_exchange.yaml" ) {

        /* With exchange */

        /* Register controllable events */
        sct->add_callback(this, sct->events["EV_moveFlock"], &CFollower::Callback_MoveFlock, NULL, NULL);
        sct->add_callback(this, sct->events["EV_moveChain"], &CFollower::Callback_MoveChain, NULL, NULL);
        sct->add_callback(this, sct->events["EV_moveStop"],  &CFollower::Callback_MoveStop,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_taskStart"], &CFollower::Callback_TaskStart, NULL, NULL);
        sct->add_callback(this, sct->events["EV_taskStop"],  &CFollower::Callback_TaskStop,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_switchF"],   &CFollower::Callback_SwitchF,   NULL, NULL);
        sct->add_callback(this, sct->events["EV_switchC"],   &CFollower::Callback_SwitchC,   NULL, NULL);
        sct->add_callback(this, sct->events["EV_switchT"],   &CFollower::Callback_SwitchT,   NULL, NULL);
        sct->add_callback(this, sct->events["EV_requestL"],  &CFollower::Callback_RequestL,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_requestC"],  &CFollower::Callback_RequestC,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_respond"],   &CFollower::Callback_Respond,   NULL, NULL);
        sct->add_callback(this, sct->events["EV_relay"],     &CFollower::Callback_Relay,     NULL, NULL);
        
        /* Register uncontrollable events */
        sct->add_callback(this, sct->events["EV_condC1"],    NULL, &CFollower::Check_CondC1,    NULL);
        sct->add_callback(this, sct->events["EV_notCondC1"], NULL, &CFollower::Check_NotCondC1, NULL);
        sct->add_callback(this, sct->events["EV_condC2"],    NULL, &CFollower::Check_CondC2,    NULL);
        sct->add_callback(this, sct->events["EV_notCondC2"], NULL, &CFollower::Check_NotCondC2, NULL);
        sct->add_callback(this, sct->events["EV_nearC"],     NULL, &CFollower::Check_NearC,     NULL);
        sct->add_callback(this, sct->events["EV_notNearC"],  NULL, &CFollower::Check_NotNearC,  NULL);
        sct->add_callback(this, sct->events["EV_condF1"],    NULL, &CFollower::Check_CondF1,    NULL);
        sct->add_callback(this, sct->events["EV_notCondF1"], NULL, &CFollower::Check_NotCondF1, NULL);
        sct->add_callback(this, sct->events["EV_condF2"],    NULL, &CFollower::Check_CondF2,    NULL);
        sct->add_callback(this, sct->events["EV_notCondF2"], NULL, &CFollower::Check_NotCondF2, NULL);
        sct->add_callback(this, sct->events["EV__respond"],  NULL, &CFollower::Check__Respond,  NULL);
        sct->add_callback(this, sct->events["EV_accept"],    NULL, &CFollower::Check_Accept,    NULL);
        sct->add_callback(this, sct->events["EV_reject"],    NULL, &CFollower::Check_Reject,    NULL);
        sct->add_callback(this, sct->events["EV__requestC"], NULL, &CFollower::Check__RequestC, NULL);
        sct->add_callback(this, sct->events["EV__start"],    NULL, &CFollower::Check__Start,    NULL);
        sct->add_callback(this, sct->events["EV__stop"],     NULL, &CFollower::Check__Stop,     NULL);
        sct->add_callback(this, sct->events["EV__message"],  NULL, &CFollower::Check__Message,  NULL);
        sct->add_callback(this, sct->events["EV__relay"],    NULL, &CFollower::Check__Relay,    NULL);
        sct->add_callback(this, sct->events["EV__exchange"], NULL, &CFollower::Check__Exchange, NULL);
        sct->add_callback(this, sct->events["EV_chosen"],    NULL, &CFollower::Check_Chosen,    NULL);
        sct->add_callback(this, sct->events["EV_notChosen"], NULL, &CFollower::Check_NotChosen, NULL);
        sct->add_callback(this, sct->events["EV_nearLF"],    NULL, &CFollower::Check_NearLF,    NULL);
        sct->add_callback(this, sct->events["EV_notNearLF"], NULL, &CFollower::Check_NotNearLF, NULL);
    
    } else {
        std::cout << "Unknown SCT file path: " << m_strSCTPath << std::endl;
    }

    Reset();
}

/****************************************/
/****************************************/

void CFollower::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    msg = CByteArray(MESSAGE_BYTE_SIZE, 255);
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

    // TEMPORARY
    // if(this->GetId() == "F1" || this->GetId() == "F6") {
    //     currentState = RobotState::TRAVELER;
    //     teamID = 255;
    //     currentMoveType = MoveType::TRAVEL;
    // }
}

/****************************************/
/****************************************/

RobotState CFollower::GetRobotState() {
    return currentState;
}

/****************************************/
/****************************************/

const std::map<UInt8, HopMsg>& CFollower::GetHops() const {
    return hopsDict;
}

/****************************************/
/****************************************/

bool CFollower::IsWorking() {
    return performingTask;
}

/****************************************/
/****************************************/

void CFollower::ControlStep() {

    std::string id = this->GetId();
    // std::cout << "\n---------- " << id << " ----------" << std::endl;

    initStepTimer++;

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/

    /* Create new msg */
    msg = CByteArray(MESSAGE_BYTE_SIZE, 255);
    msg_index = 0;

    /* Clear messages received */
    leaderMsg = Message();
    teamMsgs.clear();
    connectorMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();
    travelerMsgs.clear();

    cmsgToSend.clear();
    rmsgToSend.clear();

    leaderSignal = 255; // Default value for no signal
    hopCountToLeader = 255; // Default value for not known hop count to the leader

    /* Reset sensor reading results */
    condC2 = false;
    condF1 = false;
    condF2 = false;
    nearLF = false;
    receivedReqC   = false;
    receivedAccept = false;
    receivedReject = false;
    receivedInwardRelayMsg = false;
    receivedOutwardRelayMsg = false;
    receivedInwardSendMsg = false;
    receivedOutwardSendMsg = false;
    for(auto& info : lastBeat)
        info.second.second = 'N'; // Reset received flag to N (None)

    robotsToAccept.clear();
    nearbyTeams.clear();

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
    //std::cout << "--- Supervisors ---" << std::endl;

    if(initStepTimer > 4)
        sct->run_step();    // Run the supervisor to get the next action

    // std::cout << "[" << this->GetId() << "] " << sct->get_current_state_string() << std::endl;

    /*-----------------------------*/
    /* Implement action to perform */
    /*-----------------------------*/

    /* Set current state in msg */
    msg[msg_index++] = static_cast<UInt8>(currentState);

    /* Set sender ID in msg */
    msg[msg_index++] = stoi(id.substr(1));

    /* Set current team ID in msg */
    msg[msg_index++] = teamID;

    // if(this->GetId() == "F1") {
    //     std::cout << "state: " << (int)currentState << std::endl;
    //     std::cout << "move: " << (int)currentMoveType << std::endl;
    // }

    // Decide what to communicate depending on current state (switch between follower and connector)
    switch(currentState) {
        case RobotState::FOLLOWER: {
            //std::cout << "State: FOLLOWER" << std::endl;
            bool relaying = false;
            // for(const auto& msg : rmsgToResend) {
            //     if(msg.second.from == "L1")
            //         relaying = true;
            // }
            // if(relaying)
            //     m_pcLEDs->SetAllColors(CColor::YELLOW);
            // else
            //     m_pcLEDs->SetAllColors(teamColor[teamID]);

            m_pcLEDs->SetAllColors(CColor::GREEN);

            /* Relay task signal from leader */
            msg[msg_index++] = leaderSignal;

            /* Relay team switch signal from leader */
            msg_index += 3; // Skip to next part

            /* Hop count */
            /* Set its hop count to the leader */
            //std::cout << "Hops to leader: " << hopCountToLeader << std::endl;
            msg[msg_index++] = 1; // Number of HopMsg

            msg[msg_index++] = teamID;
            msg[msg_index++] = hopCountToLeader;
            msg_index += 2; // Skip ID

            msg_index += 4; // Skip to next part

            break;
        }
        case RobotState::CONNECTOR: {
            //std::cout << "State: CONNECTOR" << std::endl;

            bool sending = false;
            bool requesting = false;
            // for(const auto& msg : rmsgToResend) {
            //     if(msg.second.from == "L1")
            //         sending = true;
            //     // if(msg.second.type == 'R')
            //     //     requesting = true;
            // }
            // if(requesting)
            //     m_pcLEDs->SetAllColors(CColor::YELLOW);
            // else if(sending)
            //     m_pcLEDs->SetAllColors(CColor::YELLOW);
            // else
            //     m_pcLEDs->SetAllColors(CColor::CYAN);

            m_pcLEDs->SetAllColors(CColor::BLUE);

            /* Leader task signal */
            msg_index++; // Skip to next part

            /* Leader team switch signal */
            msg_index += 3; // Skip to next part

            /* Hop count */
            msg[msg_index++] = hopsDict.size(); // Set the number of HopMsg

            for(const auto& it : hopsDict) {

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
            msg_index += (2 - hopsDict.size()) * 4; // TEMP: Currently assuming only two teams

            break;
        }
        case RobotState::TRAVELER: {
            // std::cout << "State: TRAVELER" << std::endl;
            m_pcLEDs->SetAllColors(CColor::YELLOW);

            /* Leader signal */
            msg_index++; // Skip to next part

            /* Leader team switch signal */
            msg_index += 3; // Skip to next part

            /* Hop count */
            msg_index += 9; // Skip to next part
            
            break;
        }
        case RobotState::LEADER: {
            std::cerr << "State: LEADER for " << this->GetId() << ". Something went wrong." << std::endl;
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
        case MoveType::TRAVEL: {
            Travel();
            break;
        }
    }

    /* Connection Message */
    /* Set ConnectionMsg to send during this timestep */
    //std::cout << "resend size: " << cmsgToResend.size() << std::endl;
    for(auto it = cmsgToResend.begin(); it != cmsgToResend.end();) {
        if(it->first > 0) {
            if(receivedAccept || receivedReject) {
                it = cmsgToResend.erase(it); // Stop resending when a response is received
                // //std::cout << "STOP RESENDING, ACCEPT HAS BEEN RECEIVED" << std::endl;
            } else {
                cmsgToSend.push_back(it->second);
                it->first--; // Decrement timer
                ++it;
            }
        } else {
            it = cmsgToResend.erase(it);
            // //std::cout << "STOP RESENDING, TIMEOUT HAS BEEN REACHED" << std::endl;
        }
    }
    // //std::cout << "resend size: " << cmsgToResend.size() << std::endl;

    //std::cout << "cmsgToSend.size: " << cmsgToSend.size() << std::endl;
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

    /* Shared Message */
    if( !shareToLeader.empty() ) {
        msg[msg_index++] = shareToLeader[0];
        msg[msg_index++] = stoi(shareToLeader.substr(1));
    } else
        msg_index += 2;

    //std::cout << "Share to leader: " << shareToLeader << std::endl;

    if( !shareToTeam.empty() ) {
        msg[msg_index++] = shareToTeam[0];
        msg[msg_index++] = stoi(shareToTeam.substr(1));
    } else
        msg_index += 2;

    //std::cout << "Share to team: " << shareToTeam << std::endl;

    msg[msg_index++] = shareDist;

    /* Teams Nearby */
    //std::cout << "nearbyTeams.size: " << nearbyTeams.size() << std::endl;
    msg[msg_index++] = nearbyTeams.size(); // Set the number of nearby teams
    for(const auto& id : nearbyTeams) {
        msg[msg_index++] = id;
    }
    // Skip if not all bytes are used
    msg_index += (2 - nearbyTeams.size()) * 1; // TEMP: Currently assuming only two teams

    /* Relay Message */
    /* Set RelayMsg to send during this timestep */
    //std::cout << "resend size: " << rmsgToResend.size() << std::endl;
    for(auto it = rmsgToResend.begin(); it != rmsgToResend.end();) {
        if(it->first > 0) {
            rmsgToSend.push_back(it->second);
            it->first--; // Decrement timer
            ++it;
        } else {
            it = rmsgToResend.erase(it);
            // //std::cout << "STOP RESENDING, TIMEOUT HAS BEEN REACHED" << std::endl;
        }
    }

    //std::cout << "rmsgToSend.size: " << rmsgToSend.size() << std::endl;
    msg[msg_index++] = rmsgToSend.size(); // Set the number of RelayMsg
    for(const auto& relayMsg : rmsgToSend) {
        msg[msg_index++] = (UInt8)relayMsg.type;
        msg[msg_index++] = relayMsg.from[0];
        msg[msg_index++] = stoi(relayMsg.from.substr(1));
        msg[msg_index++] = (UInt8)(relayMsg.time / 256.0);
        msg[msg_index++] = (UInt8)(relayMsg.time % 256);

        if( !relayMsg.firstFollower.empty() ) {
            msg[msg_index++] = relayMsg.firstFollower[0];
            msg[msg_index++] = stoi(relayMsg.firstFollower.substr(1));
        } else
            msg_index += 2;

        msg[msg_index++] = relayMsg.robot_num;
    }
    // Skip if not all bytes are used
    msg_index += (2 - rmsgToSend.size()) * 8; // TEMP: Currently assuming only two teams

    /* Set ID of all connections to msg */
    std::vector<Message> allMsgs(teamMsgs);
    allMsgs.insert(std::end(allMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

    if( !leaderMsg.Empty() ) {
        allMsgs.push_back(leaderMsg);
    }

    //std::cout << "I saw: ";
    for(size_t i = 0; i < allMsgs.size(); i++) {
        //std::cout << allMsgs[i].ID << ", ";

        msg[msg_index++] = allMsgs[i].ID[0];    // First character of ID
        msg[msg_index++] = stoi(allMsgs[i].ID.substr(1));    // ID number

        if(i >= 29){
            std::cerr << "[" << this->GetId() << "] max connections reached" << std::endl;
            break;
        }
    }
    //std::cout << std::endl;

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

            // //std::cout << tMsgs[i].Data << std::endl;

            size_t index = 0;

            Message msg = Message();

            /* Core */
            msg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);
            msg.state = static_cast<RobotState>(tMsgs[i].Data[index++]);
            msg.ID = std::to_string(tMsgs[i].Data[index++]); // Only stores number part of the id here
            msg.teamID = tMsgs[i].Data[index++];

            /* Leader Task Signal */
            msg.leaderSignal = tMsgs[i].Data[index++];

            /* Leader Team Switch Signal */
            if(tMsgs[i].Data[index] != 255) {
                std::string switchID;
                switchID += (char)tMsgs[i].Data[index++];            // First char of ID
                switchID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                msg.robotToSwitch = switchID;
                msg.teamToJoin = tMsgs[i].Data[index++];
            } else
                index += 3;

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

                // //std::cout << "FROM: " << conMsg.from << std::endl;

                robotID = "";
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                conMsg.to = robotID;
                
                // //std::cout << "TO: " << conMsg.to << std::endl;
                
                conMsg.toTeam = tMsgs[i].Data[index++]; 

                msg.cmsg.push_back(conMsg);
            }
            index += (2 - msg_num) * 6; // TEMP: Currently assuming only two teams
            
            /* Shared Message */
            std::string robotID;
            if(tMsgs[i].Data[index] != 255) {
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                msg.shareToLeader = robotID;
            } else
                index += 2;
            
            if(tMsgs[i].Data[index] != 255) {
                robotID = "";
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                msg.shareToTeam = robotID;
            } else
                index += 2;

            msg.shareDist = tMsgs[i].Data[index++];

            /* Nearby Teams */
            msg_num = tMsgs[i].Data[index++];

            if(msg_num == 255)
                msg_num = 0;

            for(size_t j = 0; j < msg_num; j++) {
                msg.nearbyTeams.push_back(tMsgs[i].Data[index++]);
            }
            index += (2 - msg_num) * 1; // TEMP: Currently assuming only two teams

            /* Relay Message */
            msg_num = tMsgs[i].Data[index++];

            if(msg_num == 255)
                msg_num = 0;

            for(size_t j = 0; j < msg_num; j++) {

                RelayMsg relayMsg;

                relayMsg.type = (char)tMsgs[i].Data[index++];

                std::string robotID;
                robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                relayMsg.from = robotID;

                // //std::cout << "FROM: " << relayMsg.from << std::endl;
                
                relayMsg.time = tMsgs[i].Data[index++]*256 + tMsgs[i].Data[index++]; 

                if(tMsgs[i].Data[index] != 255) {
                    robotID = "";
                    robotID += (char)tMsgs[i].Data[index++];            // First char of ID
                    robotID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                    relayMsg.firstFollower = robotID;
                } else
                    index += 2;

                relayMsg.robot_num = tMsgs[i].Data[index++];

                msg.rmsg.push_back(relayMsg);
            }
            index += (2 - msg_num) * 8; // TEMP: Currently assuming only two teams

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
            } else if(msg.state == RobotState::TRAVELER) {
                msg.ID = 'F' + msg.ID;
                travelerMsgs.push_back(msg);
            }
        }
    }
}

/****************************************/
/****************************************/

void CFollower::Update() {

    //std::cout << "leaderMsg = " << ( !leaderMsg.Empty() ) << std::endl;
    //std::cout << "teamMsg = " << teamMsgs.size() << std::endl;
    //std::cout << "otherLMsg = " << otherLeaderMsgs.size() << std::endl;
    //std::cout << "otherTMsg = " << otherTeamMsgs.size() << std::endl;
    //std::cout << "connectorMsg  = " << connectorMsgs.size() << std::endl;

    if(currentState == RobotState::FOLLOWER) {

        GetLeaderInfo();
        SetConnectorToRelay();

        connectionCandidate = GetClosestNonTeam();

        if(shareToTeam.empty())  { // There are no connectors between the teams

            // Calc its distance to other team follower
            // Find if distance is the closest among other teammate messages
            // Relay shortest distance among values
            Real minDist = 255;
            if( !connectionCandidate.Empty() )
                minDist = connectionCandidate.direction.Length(); // Set its own distance to a follower in the other team
            
            for(const auto& msg : teamMsgs) {
                auto hopInfo = msg.hops;
                if(hopInfo[teamID].count > hopCountToLeader) {
                    if(msg.shareDist < minDist)
                        minDist = msg.shareDist;
                }
            }
            shareDist = (UInt8)minDist;
        }

        if( !connectionCandidate.Empty() )
            condC2 = IsClosestToRobot(connectionCandidate);

        /* Check whether it has received an accept message */
        if(currentRequest.type == 'R') {
            CheckAccept();

            /* Decrement timer */
            requestTimer--;
            //std::cout << "requestTimer: " << requestTimer << std::endl;

            /* Check whether an Accept message was not received before the timeout */
            if(requestTimer <= 0)
                receivedReject = true;
        }

        SetCMsgsToRelay();
        SetLeaderMsgToRelay(currentState);

    } else if(currentState == RobotState::CONNECTOR) {

        // TODO: Check if connections with connectors are all present
            // Extract ID in hops to a vector (no duplicate)
            // Loop connectorMsgs
                // Delete seen connector IDs
            // If no duplicate vector is empty, all connections with connectors remain
        
        // DEBUG
        // //std::cout << "--- HOPS ---" << std::endl;
        // for(const auto& it : hops) {
        //     //std::cout << "Team: " << it.first << std::endl;
        //     //std::cout << "Count: " << it.second.count << std::endl;
        //     //std::cout << "ID: " << it.second.ID << std::endl;
        // }

        CheckRequests();

        UpdateHopCounts();

        /* Find the teamIDs of followers that are within its safety range */
        for(const auto& msg : otherTeamMsgs) {
            Real dist = msg.direction.Length();
            if(nearbyTeams.count(msg.teamID) == 0) {
                if(dist < separationThres - 10) // TEMP: Added fixed buffer to distance
                    nearbyTeams.insert(msg.teamID);
            }
        }

        SetLeaderMsgToRelay(currentState);

        /* Check if there is a team that is 1 hop count away from itself */
        for(const auto& hop : hopsDict) {
            if(hop.second.count == 1) {

                // /* Ensure it is not currently sending an accept message */
                // bool sendingAccept = false;
                // for(const auto& cmsg : cmsgToResend) {
                //     if(cmsg.second.type == 'A') {
                //         sendingAccept = true;
                //         break;
                //     }
                // }
                // if( !sendingAccept )
                    condF1 = true;
            }
        }

        /* Check if all connected connectors are close to the team that this robot helps connect with */

        // Find the robot ID of connectors it's connected with
        // Check those connectors hop info in msg to see which team it is connecting for them
        // If the same teamID also exists in their nearbyTeams. If all exist, return true
        // If this is true for every connector, return true

        /* Extract connector IDs to check */
        std::set<std::string> robotIDs;

        for(const auto& hop : hopsDict) {
            if( !hop.second.ID.empty() )
                robotIDs.insert(hop.second.ID);
        }

        // If it is connected to at least one connector
        if( !robotIDs.empty() ) {

            // Find the other connector
            for(const auto& id: robotIDs) {
                for(const auto& msg : connectorMsgs) {
                    if(msg.ID == id) {

                        // If it appears in other connector's hopsDict, check if the team is also close.
                        // Find situation where its ID appears in hopsDict AND not nearby.

                        bool neededAsConnector = true;

                        // Find where it appears in hopsDict
                        for(const auto& hop : msg.hops) {
                            // std::cerr << this->GetId() << " hop of " << msg.ID << " for team " << hop.first << " is " << hop.second.ID << ", count is " << hop.second.count << std::endl;
                            if(hop.second.ID == this->GetId()) {
                                // std::cerr << this->GetId() << " appears " << msg.ID << std::endl;

                                // Check whether it is close to the team
                                // If at least one team is doen't appear, break to return false
                                // If all team is close, return true

                                for(const auto& team : msg.nearbyTeams) {
                                    if(hop.first == team) {
                                        // There is a team that's close
                                        // std::cerr << this->GetId() << " is needed by " << msg.ID << " to connect team " << team << std::endl;
                                        neededAsConnector = false;
                                        break;
                                    }
                                }
                            }
                        }
                        if(neededAsConnector) {
                            // std::cerr << this->GetId() << " NEEDED! " << std::endl;
                            condF2 = false;
                        } else {
                            // std::cerr << this->GetId() << " is not needed " << std::endl;
                            condF2 = true;
                        }
                    }
                }
            }
        }
    } else if(currentState == RobotState::TRAVELER) {

        std::vector<Message> combinedMsgs(otherTeamMsgs);
        combinedMsgs.insert(combinedMsgs.end(), otherLeaderMsgs.begin(), otherLeaderMsgs.end());

        // std::cout << "Checking if team to join has been found" << std::endl;

        /* Check whether it has reached the other team */
        for(const auto& msg : combinedMsgs) {
            if(msg.teamID == teamToJoin) {
                if(msg.direction.Length() < 50) {
                    nearLF = true;
                    // std::cout << "TEAM FOUND!" << std::endl;
                    break;
                }
            }
        }

    }
}

/****************************************/
/****************************************/

void CFollower::GetLeaderInfo() {

    /* Find the hop count to and signal from the leader */
    if( !leaderMsg.Empty() ) { // Leader is in range

        hopCountToLeader = 1;
        leaderSignal = leaderMsg.leaderSignal;
        robotToSwitch = leaderMsg.robotToSwitch;
        teamToJoin = leaderMsg.teamToJoin;

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
                robotToSwitch = teamMsgs[i].robotToSwitch;
                teamToJoin = teamMsgs[i].teamToJoin;
                break;
            }
        }
    }
}

/****************************************/
/****************************************/

Message CFollower::GetClosestNonTeam() {
    
    /* Check for the robot that this robot can connect */
    Real minDist = 10000;
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

    if(minDist < 10000)
        //std::cout << "Dist to candidate: " << minDist << std::endl;
    
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

        // //std::cout << teamMsgs[i].ID << ": ";
        // for(size_t j = 0; j < teamMsgs[i].connections.size(); j++)
        //     //std::cout << teamMsgs[i].connections[j] << ", ";
        // //std::cout << std::endl;

        // Check if the team robot has seen the non-team robot 
        if (std::find(connections.begin(), connections.end(), msg.ID) != connections.end()) {

            /* Check the distance between its candidate and nearby team robots */
            CVector2 diff = msg.direction - teamMsgs[i].direction;
            Real dist = diff.Length();

            if(dist + 2 < myDist) // TEMP: Fixed extra buffer value
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
                        receivedAccept = true;
                        currentAccept = cmsg;
                    } else                            // Request approved for another follower
                        receivedReject = true;

                } else if(cmsg.type == 'N') {         // No request has been approved
                    receivedReject = true;
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
                            receivedAccept = true;
                            currentAccept = cmsg;
                            hopsCopy = msg.hops;
                        } else                            // Request approved for another follower
                            receivedReject = true;
                    }
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CFollower::CheckRequests() {

    /* Check the shortest distance to each team */
    std::map<UInt8,Real> teamDistances;

    for(const auto& hop : hopsDict) {
        for(const auto& msg : otherTeamMsgs) {
            if(msg.teamID == hop.first && hop.second.ID.empty()) { // If hop count == 1 for a given team
                if(teamDistances.find(hop.first) == teamDistances.end()) {
                    teamDistances[hop.first] = msg.direction.Length();
                } else {
                    if(msg.direction.Length() < teamDistances[hop.first]) {
                        teamDistances[hop.first] = msg.direction.Length();
                    }
                }
            }
        }
    }

    /* Check all requests sent to itself and choose one to respond to each team */
    for(const auto& msg : otherTeamMsgs) {
        for(const auto& cmsg : msg.cmsg) {
            if(cmsg.to == this->GetId() && cmsg.type == 'R') {

                receivedReqC = true;

                /* Accept if it does not have a fixed connector (ID field is empty) */
                if(hopsDict[msg.teamID].ID.empty()) {

                    /* Accept if the distance to all robots from that team is far */
                    if(teamDistances[msg.teamID] > separationThres) {

                        /* Accept first request seen for a team */
                        if(robotsToAccept.find(msg.teamID) == robotsToAccept.end()) {
                            robotsToAccept[msg.teamID] = msg;
                            continue;
                        }

                        Real currentDist = robotsToAccept[msg.teamID].direction.Length();
                        Real newDist = msg.direction.Length();

                        /* Send an accept message to the closest follower */
                        if(newDist < currentDist)
                            robotsToAccept[msg.teamID] = msg;
                    }
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
            auto teamHops = msg.hops;

            /* Relay Request message received from robots with greater hop count */
            if( !receivedRequest ) {
                if(cmsg.type == 'R' && teamHops[teamID].count > hopCountToLeader) {
                    if(currentRequest.type != 'R') { // Check if it is currently not requesting
                        cmsgToSend.push_back(cmsg);
                        receivedRequest = true;
                        //std::cout << "Relay Request, from: " << cmsg.from << " to: " << cmsg.to << std::endl;
                    }
                }
            }
            
            /* Relay Accept message received from robots with smaller hop count */
            if( !receivedAccept ) {
                if(cmsg.type == 'A' && teamHops[teamID].count < hopCountToLeader) {
                    cmsgToSend.push_back(cmsg);
                    receivedAccept = true;
                    //std::cout << "Relay Accept, from: " << cmsg.from << " to: " << cmsg.to << std::endl;
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CFollower::SetLeaderMsgToRelay(const RobotState state) {
    if(state == RobotState::FOLLOWER) {

        std::vector<Message> inwardMsgs;
        std::vector<Message> outwardMsgs;
        
        // Add connectors and leaders/followers from other teams to check for inward message relay
        inwardMsgs.insert(std::end(inwardMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
        inwardMsgs.insert(std::end(inwardMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
        inwardMsgs.insert(std::end(inwardMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));

        // Add the leader to check for outward message relay
        if( !leaderMsg.Empty() )
            outwardMsgs.push_back(leaderMsg);

        // Split messages from team followers into two groups.
        for(const auto& msg : teamMsgs) {
            auto teamHops = msg.hops;
            if( !msg.rmsg.empty() ) {
                if(teamHops[teamID].count > hopCountToLeader)
                    inwardMsgs.push_back(msg);
                if(teamHops[teamID].count < hopCountToLeader)
                    outwardMsgs.push_back(msg);
            }
        }

        // For inward message, find all that's not in resend
        for(auto& msg : inwardMsgs) {
            for(auto& relayMsg : msg.rmsg) {
                UInt8 receivedTeamID = stoi(relayMsg.from.substr(1));
                if(receivedTeamID != teamID) {
                    if(lastBeat.find(receivedTeamID) == lastBeat.end()) { // If its the first time receiving, add it to lastBeat received
                        if(msg.state == RobotState::LEADER) {
                            relayMsg.firstFollower = this->GetId();
                            lastBeat[receivedTeamID] = {relayMsg,'L'};
                        } else if(msg.teamID != teamID) {
                            relayMsg.firstFollower = this->GetId();
                            lastBeat[receivedTeamID] = {relayMsg,'F'};
                        } else
                            lastBeat[receivedTeamID] = {relayMsg,'F'};
                    } else {
                        if(relayMsg.time > lastBeat[receivedTeamID].first.time) { // Else update it only if the timestep is newer
                            if(msg.state == RobotState::LEADER) {
                                relayMsg.firstFollower = this->GetId();
                                lastBeat[receivedTeamID] = {relayMsg,'L'};
                            } else if(msg.teamID != teamID) {
                                relayMsg.firstFollower = this->GetId();
                                lastBeat[receivedTeamID] = {relayMsg,'F'};
                            } else
                                lastBeat[receivedTeamID] = {relayMsg,'F'};
                        }
                    }
                }
            }
        }

        // For outward message, only find one
        for(const auto& msg : outwardMsgs) {
            for(const auto& relayMsg : msg.rmsg) {
                if(stoi(relayMsg.from.substr(1)) == teamID) {
                    if(lastBeat.find(teamID) == lastBeat.end()) { // If its the first time receiving, add it to lastBeat received
                        if(msg.state == RobotState::LEADER)
                            lastBeat[teamID] = {relayMsg,'L'};
                        else
                            lastBeat[teamID] = {relayMsg,'F'};
                    } else {
                        if(relayMsg.time > lastBeat[teamID].first.time) { // Else update it only if the timestep is newer
                            if(msg.state == RobotState::LEADER)
                                lastBeat[teamID] = {relayMsg,'L'};
                            else
                                lastBeat[teamID] = {relayMsg,'F'};
                        }
                    }
                }
            }
        }

    } else if(state == RobotState::CONNECTOR) {

        /* Check whether new relayMsg is received */
        for(const auto& hop : hopsDict) {
            /* Check the team */
            if(hop.second.count == 1) {

                std::vector<Message> combinedMsgs(otherLeaderMsgs);
                combinedMsgs.insert(std::end(combinedMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

                for(const auto& msg : combinedMsgs) {
                    if(msg.teamID == hop.first) {
                        for(const auto& relayMsg : msg.rmsg) {
                            if(stoi(relayMsg.from.substr(1)) == hop.first) {
                                if(lastBeat.find(hop.first) == lastBeat.end()) { // If its the first time receiving, add it to lastBeat received
                                    if(msg.state == RobotState::LEADER)
                                        lastBeat[hop.first] = {relayMsg,'L'};
                                    else
                                        lastBeat[hop.first] = {relayMsg,'F'};
                                } else {
                                    if(relayMsg.time > lastBeat[hop.first].first.time) { // Else update it only if the timestep is newer
                                        if(msg.state == RobotState::LEADER)
                                            lastBeat[hop.first] = {relayMsg,'L'};
                                        else
                                            lastBeat[hop.first] = {relayMsg,'F'};
                                    }
                                }
                            }
                        }
                    }
                }
            /* Check the connectors */
            } else {
                for(const auto& msg : connectorMsgs) {
                    if(hop.second.ID == msg.ID) {
                        //std::cerr << "---------" << this->GetId() << "---------" << std::endl;
                        for(const auto& relayMsg : msg.rmsg) {
                            if(stoi(relayMsg.from.substr(1)) == hop.first) {
                                //std::cerr << msg.ID << "(" << hop.first << ") -> " << this->GetId() << std::endl;
                                if(lastBeat.find(hop.first) == lastBeat.end()) { // If its the first time receiving, add it to lastBeat received
                                    lastBeat[hop.first] = {relayMsg,'F'};
                                    //std::cerr << "Relaying this " << relayMsg.time << std::endl;
                                } else {
                                    if(relayMsg.time > lastBeat[hop.first].first.time) { // Else update it only if the timestep is newer
                                        lastBeat[hop.first] = {relayMsg,'F'};
                                        //std::cerr << " Relaying this " << relayMsg.time << std::endl;
                                    }
                                }
                            }
                        }
                    }
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

    /* Add every other visible team to hop map */
    for(const auto& msg : otherTeamMsgs) {

        /* Add hop count entry if not yet registered */
        if(hopsDict.find(msg.teamID) == hopsDict.end()) {
            HopMsg hop;
            hop.count = 1;
            hopsDict[msg.teamID] = hop;
        }
    }

    /* Extract connector IDs to check */
    std::set<std::string> robotIDs;

    for(const auto& hop : hopsDict) {
        if( !hop.second.ID.empty() )
            robotIDs.insert(hop.second.ID);
    }

    /* Extract Messages from connectors that have the IDs found previously */
    std::map<std::string, Message> robotMessages;

    for(const auto& msg : connectorMsgs) {

        if(robotIDs.empty())
            break;

        /* Find the next connector */
        if(robotIDs.count(msg.ID)) { // Should always return 0 or 1 as it is a set
            robotMessages[msg.ID] = msg;
            robotIDs.erase(msg.ID);
        }
    }

    /* If a connector was not found, update hop count if it has become a follower */
    if( !robotIDs.empty() ) {
        for(const auto& msg : otherTeamMsgs) {

            /* Robot is found to be a follower so delete entries from hopsDict with the robot's id */
            if(robotIDs.count(msg.ID)) { // Should always return 0 or 1 as it is a set

                /* Check if it is not a robot that it has just sent an accept message to */
                bool sentAccept = false;
                for(const auto& sendMsg : cmsgToResend) {
                    if(sendMsg.second.to == msg.ID) {
                        sentAccept = true;
                        robotIDs.erase(msg.ID);
                        break;
                    }
                }

                if( !sentAccept ) {
                    /* Find all keys that this robot appears in */
                    std::vector<UInt8> teamKeys;
                    for(const auto& hop : hopsDict) {
                        if(hop.second.ID == msg.ID)
                            teamKeys.push_back(hop.first);
                    }

                    /* Delete the robot's ID and update hop count to 1 */
                    for(const auto& key : teamKeys) {
                        hopsDict[key].ID = "";
                        hopsDict[key].count = 1;
                    }

                    robotIDs.erase(msg.ID);
                }
            }
        }
    }

    if( !robotIDs.empty() ) {
        std::cerr << "robotIDs not empty for robot: " << this->GetId() << std::endl;
    }

    /* Update hop count */
    for(auto& hop : hopsDict) {
        std::string previousRobotID = hop.second.ID;

        if( !previousRobotID.empty() ) {

            if(robotIDs.count(previousRobotID)) {
                hop.second.count = 255; // Could not be found, so set it to max value
            } else {
                // If it has just send an accept, don't update count
                UInt8 teamToCheck = hop.first;
                HopMsg previousHop = robotMessages[previousRobotID].hops[teamToCheck];

                bool sendingAccept = false;
                for(const auto& pair : cmsgToResend) {
                    if(pair.second.type == 'A' && pair.second.to == previousRobotID) {
                        sendingAccept = true;
                    }
                }
                if( !sendingAccept )
                hop.second.count = previousHop.count + 1; // Increment by 1
            }
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
    CVector2 sumForce      = teamWeight*teamForce + robotWeight*robotForce + obstacleWeight*obstacleForce;

    /* DEBUGGING */
    if(this->GetId() == "F1") {
        //std::cout << "team: " << teamForce.Length() << std::endl;
        //std::cout << "robot: " << robotForce.Length() << std::endl;
        //std::cout << "obstacle: " << obstacleForce.Length() << std::endl;
        //std::cout << "sum: " << sumForce.Length() << std::endl;
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
    // if(hopCountToLeader < 255) {
    //     UInt8 minCount = hopCountToLeader - 1;

    //     for(size_t i = 0; i < teamMsgs.size(); i++) {
    //         if(teamMsgs[i].hops[teamID].count > minCount)
    //             repulseMsgs.push_back(teamMsgs[i]);
    //     }
    // } else {
    //     repulseMsgs.insert(std::end(repulseMsgs), std::begin(teamMsgs), std::end(teamMsgs));
    // }
    
    if( !leaderMsg.Empty() )
        repulseMsgs.push_back(leaderMsg);
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(teamMsgs), std::end(teamMsgs));

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
            // Real length = (fProxReads[i] - 0.9) * m_sWheelTurningParams.MaxSpeed * 10; // Map length to 0 ~ max_speed
            // Real length = fProxReads[i] * m_sWheelTurningParams.MaxSpeed;

            Real distance = -( log(fProxReads[i]) / log(exp(1)) );
            Real length = (0.1 - distance) / 0.1 * m_sWheelTurningParams.MaxSpeed;
            vec = CVector2(length, PROX_ANGLE[i]);
            
            resVec -= vec; // Subtract because we want the vector to repulse from the obstacle
        }
        // if(this->GetId() == "F1") {
        //     std::cout << "raw " << i << ": " << fProxReads[i] << std::endl;
        //     // std::cout << "sensor " << i << ": " << (fProxReads[i] - 0.9) * m_sWheelTurningParams.MaxSpeed * 10 << std::endl;
        //     // std::cout << "sensor " << i << ": " << -( log(fProxReads[i]) / log(exp(1)) / 0.1)/*  * m_sWheelTurningParams.MaxSpeed */ << std::endl;
        // }
    }

    resVec /= 8;

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    // if(this->GetId() == "F1") {
    //     std::cout << "vector: " << resVec << std::endl;
    //     std::cout << "length: " << resVec.Length() << std::endl;

    // }

    return resVec;
    // return CVector2();
}

/****************************************/
/****************************************/

void CFollower::Travel() {
    /* Calculate overall force applied to the robot */
    CVector2 travelForce   = GetChainTravelVector();
    CVector2 robotForce    = GetRobotRepulsionVector();
    CVector2 obstacleForce = GetObstacleRepulsionVector();

    CVector2 sumForce      = teamWeight*travelForce + 0.3*robotForce + 0.5*obstacleWeight*obstacleForce;

    /* Set Wheel Speed */
    if(travelForce.Length() > 0.0f)
        SetWheelSpeedsFromVector(sumForce);
    else
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    
    /* Set Wheel Speed */
    SetWheelSpeedsFromVector(sumForce);
}

/****************************************/
/****************************************/

CVector2 CFollower::GetChainTravelVector() {

    CVector2 resVec = CVector2();

    /* Sort connectors according to the hop count towards the target team (large -> small) */
    auto property = teamToJoin;
    auto sortRuleLambda = [property] (Message& m1, Message& m2) -> bool
    {
        return m1.hops[property].count > m2.hops[property].count;
    };

    std::vector<Message> sortedConnectorMsgs(connectorMsgs);
    std::sort(sortedConnectorMsgs.begin(), sortedConnectorMsgs.end(), sortRuleLambda);

    // for(auto& msg : sortedConnectorMsgs) {
    //     std::cout << msg.hops[teamToJoin].count << std::endl;
    // }

    /* Find the next connector to move towards */
    Message nextConnector;
    for(auto& msg : sortedConnectorMsgs) {
        if(nextConnector.Empty())
            nextConnector = msg;
        else {
            
            /* Check distance to the chain */
            if(msg.direction.Length() < 70) {

                /* Calculate target vector */
                CVector2 margin = msg.direction;
                margin.Rotate(CRadians::PI_OVER_TWO);
                margin.Normalize();
                margin *= 20;
                CVector2 target = msg.direction + margin;

                /* Check whether its movement will cross a connection between connectors */
                bool noIntersection = true;
                for(const auto& hop : msg.hops) {
                    std::string id = hop.second.ID;
                    CVector2 otherConnectorVec = CVector2();
                    for(const auto& hopMsg : connectorMsgs) {
                        if(hopMsg.ID == id)
                            otherConnectorVec = hopMsg.direction;
                    }

                    if(otherConnectorVec.Length() == 0.0f) {
                        // std::cout << "No intersection (Not found)" << std::endl;
                    } else {
                        // std::cout << "Target: " << target << std::endl;
                        // std::cout << "Start: " << msg.direction << std::endl;
                        // std::cout << "End: " << otherConnectorVec << std::endl;

                        if(isIntersect(target, msg.direction, otherConnectorVec)) {
                            // std::cout << "Intersect" << std::endl;
                            noIntersection = false;
                        } else {
                            // std::cout << "No intersection" << std::endl;
                        }
                    }
                }

                if(noIntersection)
                    nextConnector = msg;
            }
        }
    }

    // std::cout << "Next connector: " << nextConnector.ID << std::endl;

    /* Calculate the position of the left side of the connector */

    // std::cout << "direction: " << nextConnector.direction << std::endl;

    CVector2 margin = nextConnector.direction;
    margin.Rotate(CRadians::PI_OVER_TWO);

    // std::cout << "rotated: " << margin << std::endl;

    margin.Normalize();
    margin *= 20;

    resVec = nextConnector.direction + margin;

    // std::cout << "margin.Length: " << margin.Length() << std::endl;
    // std::cout << "direction.Length: " << nextConnector.direction.Length() << std::endl;
    // std::cout << "margin: " << margin << std::endl;
    // std::cout << "target: " << resVec << std::endl;

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
    //std::cout << "[" << this->GetId() << "] ";
}

/****************************************/
/****************************************/

/* Callback functions (Controllable events) */

void CFollower::Callback_TaskStart(void* data) {
    //std::cout << "Action: taskStart" << std::endl;
    performingTask = true;
}

void CFollower::Callback_TaskStop(void* data) {
    //std::cout << "Action: taskStop" << std::endl;
    performingTask = false;
}

void CFollower::Callback_MoveFlock(void* data) {
    //std::cout << "Action: moveFlock" << std::endl;
    currentMoveType = MoveType::FLOCK;
    currentRequest = ConnectionMsg(); // Clear any existing requests
}

void CFollower::Callback_MoveChain(void* data) {
    //std::cout << "Action: moveChain" << std::endl;
    currentMoveType = MoveType::TRAVEL;
    currentRequest = ConnectionMsg(); // Clear any existing requests
}

void CFollower::Callback_MoveStop(void* data) {
    //std::cout << "Action: moveStop" << std::endl;
    currentMoveType = MoveType::STOP;
}

void CFollower::Callback_SwitchF(void* data) {
    //std::cout << "Action: switchF" << std::endl;

    /* Set new teamID */
    if(currentState == RobotState::CONNECTOR) {
        for(const auto& hop : hopsDict) {
            if(hop.second.count == 1) {
                teamID = hop.first;
                break;
            }
        }
        hopsDict.clear();

    } else if(currentState == RobotState::TRAVELER) {
        teamID = teamToJoin;
    }

    // std::cout << "JOINING TEAM" << std::endl;

    currentState = RobotState::FOLLOWER;
}

void CFollower::Callback_SwitchC(void* data) {
    //std::cout << "Action: switchC" << std::endl;
    
    if(currentAccept.from[0] == 'L') {  // Accept received from the leader

        /* Add every other visible team to hop map */
        for(const auto& msg : otherTeamMsgs) {

            /* Add hop count entry if not yet registered */
            if(hopsDict.find(msg.teamID) == hopsDict.end()) {
                HopMsg hop;
                hop.count = 1;
                hopsDict[msg.teamID] = hop;
            }
        }
    } else {    // Accept received from a connector

        /* Use the connector to generate its hop count to other teams */
        hopsCopy.erase(teamID);            // Delete entry of its own team

        for(auto& it : hopsCopy) {         // Loop to add hop count of 1 to each item
            it.second.count++;
            it.second.ID = currentAccept.from;
        }
        hopsDict = hopsCopy;                   // Set to its hops
    }

    /* Set hop count to the team it is leaving to 1 */
    HopMsg hop;
    hop.count = 1;
    hopsDict[teamID] = hop;

    /* Reset variables */
    currentAccept = ConnectionMsg(); 
    hopsCopy.clear();
    shareToLeader = "";
    shareToTeam = "";
    shareDist = 255;

    currentState = RobotState::CONNECTOR;
    teamID = 255;
    setCTriggered = true;
}

void CFollower::Callback_SwitchT(void* data) {
    //std::cout << "Action: switchT" << std::endl;

    /* Reset variables */
    shareToLeader = "";
    shareToTeam = "";
    shareDist = 255;

    currentState = RobotState::TRAVELER;
    teamID = 255;
}

void CFollower::Callback_RequestL(void* data) {
    //std::cout << "Action: RequestL" << std::endl;

    /* Set request to send */
    ConnectionMsg cmsg;
    cmsg.type   = 'R';
    cmsg.from   = this->GetId();
    cmsg.to     = "L" + std::to_string(teamID);
    cmsg.toTeam = teamID;
    cmsgToResend.push_back({sendDuration,cmsg}); // Transmit public event

    currentRequest = cmsg;
    requestTimer = waitRequestDuration;
    //std::cout << "requestTimer SET: " << requestTimer << std::endl;
}

void CFollower::Callback_RequestC(void* data) {
    //std::cout << "Action: RequestC" << std::endl;

    /* Set request to send */
    ConnectionMsg cmsg;
    cmsg.type   = 'R';
    cmsg.from   = this->GetId();
    cmsg.to     = connectionCandidate.ID;
    cmsg.toTeam = 255; // No team
    cmsgToResend.push_back({sendDuration,cmsg}); // Transmit public event

    currentRequest = cmsg;
    requestTimer = waitRequestDuration;
    //std::cout << "requestTimer SET: " << requestTimer << std::endl;
}

void CFollower::Callback_Respond(void* data) {
    //std::cout << "Action: Respond" << std::endl;

    for(const auto& it : robotsToAccept) {
        ConnectionMsg cmsg;
        cmsg.type   = 'A';
        cmsg.from   = this->GetId();
        cmsg.to     = it.second.ID;
        cmsg.toTeam = it.first;
        cmsgToResend.push_back({sendDuration,cmsg}); // Transmit public event

        /* Update hop count to the team using the new connector */
        hopsDict[it.first].count++; // 1 -> 2
        hopsDict[it.first].ID = it.second.ID;
    }
}

void CFollower::Callback_Relay(void* data) {
    //std::cout << "Action: relay" << std::endl;

    for(const auto& info : lastBeat) {

        /* Remove any existing old messages from the same team that is current being sent */
        for(auto it = rmsgToResend.begin(); it != rmsgToResend.end();) {
            if(it->first == info.first) {
                it = rmsgToResend.erase(it);
            } else {
                ++it;
            }
        }

        if(info.second.second != 'N')
            rmsgToResend.push_back({sendDuration,info.second.first});
    }
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CFollower::Check__Start(void* data) {
    if( !leaderMsg.Empty() && leaderMsg.leaderSignal == 1) {
        // std::cout << "Event: " << 1 << " - _start" << std::endl;
        return 1;
    }
    // std::cout << "Event: " << 0 << " - _start" << std::endl;
    return 0;
}

unsigned char CFollower::Check__Stop(void* data) {
    if( !leaderMsg.Empty() && leaderMsg.leaderSignal == 0) {
        // std::cout << "Event: " << 1 << " - _stop" << std::endl;
        return 1;
    }
    // std::cout << "Event: " << 0 << " - _stop" << std::endl;
    return 0;
}

unsigned char CFollower::Check_CondC1(void* data) {
    if(connectionCandidate.direction.Length() >= separationThres) {
        // std::cout << "Event: " << 1 << " - condC1" << std::endl;
        return 1;
    }
    // std::cout << "Event: " << 0 << " - condC1" << std::endl;
    return 0;
}

unsigned char CFollower::Check_NotCondC1(void* data) {
    if(connectionCandidate.direction.Length() >= separationThres) {
        // std::cout << "Event: " << 0 << " - notCondC1" << std::endl;
        return 0;
    }
    // std::cout << "Event: " << 1 << " - notCondC1" << std::endl;
    return 1;
}

unsigned char CFollower::Check_CondC2(void* data) {
    // std::cout << "Event: " << condC2 << " - condC2" << std::endl;
    return condC2;
}

unsigned char CFollower::Check_NotCondC2(void* data) {
    // std::cout << "Event: " << !condC2 << " - notCondC2" << std::endl;
    return !condC2;
}

// unsigned char CFollower::Check_CondC3(void* data) {
//     if( !connectionCandidate.Empty()) {
//         if(teamID < connectionCandidate.teamID) {
//             // std::cout << "Event: " << 1 << " - condC3" << std::endl;
//             return 1;
//         }
//     }
//     // std::cout << "Event: " << 0 << " - condC3" << std::endl;
//     return 0;
// }

// unsigned char CFollower::Check_NotCondC3(void* data) {
//     if( !connectionCandidate.Empty() ) {
//         if(teamID < connectionCandidate.teamID) {
//             // std::cout << "Event: " << 0 << " - notCondC3" << std::endl;
//             return 0;
//         }
//     }
//     // std::cout << "Event: " << 1 << " - notCondC3" << std::endl;
//     return 1;
// }

unsigned char CFollower::Check_NearC(void* data) {
    bool connectorSeen = !connectorMsgs.empty();
    // std::cout << "Event: " << connectorSeen << " - nearC" << std::endl;
    return connectorSeen;
}

unsigned char CFollower::Check_NotNearC(void* data) {
    bool connectorSeen = !connectorMsgs.empty();
    // std::cout << "Event: " << !connectorSeen << " - notNearC" << std::endl;
    return !connectorSeen;
}

unsigned char CFollower::Check_CondF1(void* data) {
    // std::cout << "Event: " << condF1 << " - condF1" << std::endl;
    return condF1;
}

unsigned char CFollower::Check_NotCondF1(void* data) {
    // std::cout << "Event: " << !condF1 << " - notCondF1" << std::endl;
    return !condF1;
}

unsigned char CFollower::Check_CondF2(void* data) {
    // std::cout << "Event: " << condF2 << " - condF2" << std::endl;
    return condF2;
}

unsigned char CFollower::Check_NotCondF2(void* data) {
    // std::cout << "Event: " << !condF2 << " - notCondF2" << std::endl;
    return !condF2;
}

unsigned char CFollower::Check__RequestC(void* data) {
    // std::cout << "Event: " << receivedReqC << " - _requestC" << std::endl;
    return receivedReqC;
}

unsigned char CFollower::Check__Respond(void* data) {
    // std::cout << "Event: " << (receivedAccept || receivedReject) << " - _respond" << std::endl;
    return receivedAccept || receivedReject;
}

unsigned char CFollower::Check_Accept(void* data) {
    // std::cout << "Event: " << receivedAccept << " - accept" << std::endl;
    return receivedAccept;
}

unsigned char CFollower::Check_Reject(void* data) {
    // std::cout << "Event: " << receivedReject << " - reject" << std::endl;
    return receivedReject;
}

unsigned char CFollower::Check__Message(void* data) {
    for(const auto& info : lastBeat) {
        if(info.second.second == 'L') {
            // std::cout << "Event: " << 1 << " - _message" << std::endl;
            return 1;
        }
    }
    // std::cout << "Event: " << 0 << " - _message" << std::endl;
    return 0;
}

unsigned char CFollower::Check__Relay(void* data) {
    for(const auto& info : lastBeat) {
        if(info.second.second == 'F') {
            // std::cout << "Event: " << 1 << " - _relay" << std::endl;
            return 1;
        }
    }
    // std::cout << "Event: " << 0 << " - _relay" << std::endl;
    return 0;
}

unsigned char CFollower::Check__Exchange(void* data) {
    bool receivedExchange = !robotToSwitch.empty();
    // std::cout << "Event: " << receivedExchange << " - _exchange" << std::endl;
    return receivedExchange;
}

unsigned char CFollower::Check_Chosen(void* data) {
    bool chosen = robotToSwitch == this->GetId();
    // std::cout << "Event: " << chosen << " - chosen" << std::endl;
    return chosen;
}

unsigned char CFollower::Check_NotChosen(void* data) {
    bool chosen = robotToSwitch == this->GetId();
    // std::cout << "Event: " << !chosen << " - notChosen" << std::endl;
    return !chosen;
}

unsigned char CFollower::Check_NearLF(void* data) {
    // std::cout << "Event: " << nearLF << " - nearLF" << std::endl;
    return nearLF;
}

unsigned char CFollower::Check_NotNearLF(void* data) {
    // std::cout << "Event: " << !nearLF << " - notNearLF" << std::endl;
    return !nearLF;
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
