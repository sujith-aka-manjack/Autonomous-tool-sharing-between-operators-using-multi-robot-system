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
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"         );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing" );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing" );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds");

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
        GetNodeAttribute(GetNode(t_node, "team"), "to_chain_threshold", toChainThreshold);
        GetNodeAttribute(GetNode(t_node, "team"), "to_follow_threshold", toFollowThreshold);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Initialization */
    currentState = RobotState::FOLLOWER; // Set initial state to follower
    performingTask = false; // Robot initially not working on any task
    joinChainTriggered = false; // Initialize flag
    connection_timer = 0; // Init timer used to determine whether to connect

    /*
    * Init SCT Controller
    */
    sct = new SCTProb();
    sct->add_callback(this, EV_flock,      &CFollower::Callback_Flock,      NULL, NULL);
    sct->add_callback(this, EV_stop,       &CFollower::Callback_Stop,       NULL, NULL);
    sct->add_callback(this, EV_startTask,  &CFollower::Callback_StartTask,  NULL, NULL);
    sct->add_callback(this, EV_stopTask,   &CFollower::Callback_StopTask,   NULL, NULL);
    sct->add_callback(this, EV_joinLeader, &CFollower::Callback_JoinLeader, NULL, NULL);
    sct->add_callback(this, EV_joinChain,  &CFollower::Callback_JoinChain,  NULL, NULL);

    sct->add_callback(this, EV_taskEnded,         NULL, &CFollower::Check_TaskEnded,         NULL);
    sct->add_callback(this, EV_getStart,          NULL, &CFollower::Check_GetStart,          NULL);
    sct->add_callback(this, EV_getStop,           NULL, &CFollower::Check_GetStop,           NULL);
    sct->add_callback(this, EV_chainNear,         NULL, &CFollower::Check_ChainNear,         NULL);
    sct->add_callback(this, EV_chainFar,          NULL, &CFollower::Check_ChainFar,          NULL);
    sct->add_callback(this, EV_leaderNear,        NULL, &CFollower::Check_LeaderNear,        NULL);
    sct->add_callback(this, EV_leaderFar,         NULL, &CFollower::Check_LeaderFar,         NULL);
    sct->add_callback(this, EV_closestToChain,    NULL, &CFollower::Check_ClosestToChain,    NULL);
    sct->add_callback(this, EV_notClosestToChain, NULL, &CFollower::Check_NotClosestToChain, NULL);

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
    msg = CByteArray(16, 255);
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
    msg = CByteArray(16, 255);
    msg_index = 0;

    /* Clear messages received */
    leaderMsg = Message();
    teamMsgs.clear();
    chainMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();

    /* Reset sensor reading results */
    minNonTeamDistance = __FLT_MAX__;
    isClosestToOther = false;
    closestLeaderDistance = __FLT_MAX__;
    closestLeader = 255;    // No leader
    // connections.clear();

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();

    /*------------------------*/
    /* Update sensor readings */
    /*------------------------*/
    UpdateSensors();

    /*--------------------*/
    /* Run SCT controller */
    /*--------------------*/
    
    sct->run_step();

    // PrintName();
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

    // Decide what to communicate depending on current state (switch between follower and chain)
    switch(currentState) {
        case RobotState::FOLLOWER: {
            std::cout << "State: FOLLOWER" << std::endl;
            m_pcLEDs->SetAllColors(teamColor[teamID]);

            /* Set current team ID in msg */
            msg[msg_index++] = teamID;

            /* Set how many non-team members it has seen */
            msg[msg_index++] = chainMsgs.size() + otherLeaderMsgs.size() + otherTeamMsgs.size();
            break;
        }
        case RobotState::CHAIN: {
            std::cout << "State: CHAIN" << std::endl;
            m_pcLEDs->SetAllColors(CColor::CYAN);

            /* Set current team ID in msg */
            msg[msg_index++] = teamID;

            // /* Set connected chain entity info to message (up to 6) */
            // for(int i = 0; i < connections.size(); i++) {
            //     msg[msg_index++] = connections[i][0];    // First character of ID
            //     msg[msg_index++] = stoi(connections[i].substr(1));    // ID number

            //     if(i == 5)
            //         break;
            // }
            break;
        }
        case RobotState::LEADER: {
            std::cout << "State: LEADER. Something went wrong." << std::endl;
            break;
        }
    }

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

                // /* Store which chain entities the other chain robot is connected to */
                // while(tMsgs[i].Data[index] != 255) {    // Check if data exists
                //     std::string chainID;
                //     chainID += (char)tMsgs[i].Data[index++];            // First char of ID
                //     chainID += std::to_string(tMsgs[i].Data[index++]);  // ID number
                //     msg.connections.push_back(chainID);
                // }
                
                // sort(std::begin(msg.connections), std::end(msg.connections));
                chainMsgs.push_back(msg);
            } 
            /* Message from team */
            else if(msg.teamid == teamID) {
                /* Message from leader */
                if(msg.state == RobotState::LEADER) {
                    msg.id = 'L' + msg.id;
                    msg.numOtherTeamSeen = tMsgs[i].Data[index++];
                    msg.taskSignal = tMsgs[i].Data[index++];
                    std::cout << "Signal: " << msg.taskSignal << std::endl;
                    leaderMsg = msg;
                } 
                /* Message from follower */
                else if(msg.state == RobotState::FOLLOWER) {
                    msg.id = 'F' + msg.id;
                    msg.numOtherTeamSeen = tMsgs[i].Data[index++];
                    teamMsgs.push_back(msg);
                }
            } 
            /* Message from other team */
            else {
                /* Message from other leader */
                if(msg.state == RobotState::LEADER) {
                    msg.id = 'L' + msg.id;
                    msg.numOtherTeamSeen = tMsgs[i].Data[index++];
                    otherLeaderMsgs.push_back(msg);   
                }
                /* Message from other follower */
                else if(msg.state == RobotState::FOLLOWER) {
                    msg.id = 'F' + msg.id;
                    msg.numOtherTeamSeen = tMsgs[i].Data[index++];
                    otherTeamMsgs.push_back(msg); 
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CFollower::UpdateSensors() {

    /* Check leader distance with chain (including other leader) */

    std::cout << "chainMsg  = " << chainMsgs.size() << std::endl;
    std::cout << "otherLMsg = " << otherLeaderMsgs.size() << std::endl;

    /* Combine messages received from all non-team entities */
    std::vector<Message> nonTeamMsgs(chainMsgs);
    nonTeamMsgs.insert(std::end(nonTeamMsgs),
                       std::begin(otherLeaderMsgs),
                       std::end(otherLeaderMsgs));
    nonTeamMsgs.insert(std::end(nonTeamMsgs),
                       std::begin(otherTeamMsgs),
                       std::end(otherTeamMsgs));

    if(currentState == RobotState::FOLLOWER) {

        /* Check the distance to the closest non-team entity */
        /* Event: chainNear, chainFar */
        for(int i = 0 ; i < nonTeamMsgs.size(); i++) {
            Real dist = nonTeamMsgs[i].direction.Length();
            if(dist < minNonTeamDistance)
                minNonTeamDistance = dist;
        }

        std::cout << "Dist to non-team: " << minNonTeamDistance << std::endl;

        /* Check if any team member is closer to a non-team member and has seen a non-team member */
        /* Event: closestToChain, notClosestToChain */
        /* Add leader to team messages*/
        std::vector<Message> combinedTeamMsgs(teamMsgs);
        if(leaderMsg.direction.Length() > 0.0f)
            combinedTeamMsgs.push_back(leaderMsg);

        /* Was any non-team member detected? Cannot be closest to non-team member if none were detected */
        if(nonTeamMsgs.empty())
            isClosestToOther = false;
        else
            isClosestToOther = true;
        
        Real minDist = __FLT_MAX__;
        std::cout << "# size: " << combinedTeamMsgs.size() << std::endl;
        for(int i = 0; i < combinedTeamMsgs.size(); i++) {
            for(int j = 0; j < nonTeamMsgs.size(); j++) {
                CVector2 diff = combinedTeamMsgs[i].direction - nonTeamMsgs[j].direction;
                Real dist = diff.Length();
                std::cout << "## " << combinedTeamMsgs[i].id << " dist: " << dist << std::endl;
                std::cout << "## " << combinedTeamMsgs[i].id << " seen: " << combinedTeamMsgs[i].numOtherTeamSeen << std::endl;
                if(dist < minDist && combinedTeamMsgs[i].numOtherTeamSeen > 0) {
                    minDist = dist;
                    if(minDist < minNonTeamDistance) {
                        isClosestToOther = false;   // There is a team member that is closer to a non-team member
                        break;
                    }
                }
            }
            if( !isClosestToOther )
                break;
        }

        /* Make robot wait for 3 timesteps before commiting to become a chain member */
        if(isClosestToOther && connection_timer < 3) {
            connection_timer++;
            std::cout << "TIMER INCREMENT " << connection_timer << std::endl;
            isClosestToOther = false;
        }
        
    }
    else if(currentState == RobotState::CHAIN) {
        /* Check the distance to the closest leader */
        /* Event: leaderNear, leaderFar */
        for(int i = 0 ; i < otherLeaderMsgs.size(); i++) {
            Real dist = otherLeaderMsgs[i].direction.Length();
            if(dist < closestLeaderDistance) {
                closestLeaderDistance = dist;
                closestLeader = stoi(otherLeaderMsgs[i].id.substr(1));
            }
        }
        
        std::cout << "Dist to leader(" << closestLeader << "): " << closestLeaderDistance << std::endl;
        
        /* Check if any leader is closer to another team (other team and chain) than itself and has seen another team */
        /* Event: closestToChain, notClosestToChain */
        for(int i = 0; i < otherLeaderMsgs.size(); i++) {

            /* Remove other leader and its team currently being considered from nonTeamMsgs */
            std::vector<Message> tempOtherMsgs(nonTeamMsgs);
            auto itr = tempOtherMsgs.begin();
            while(itr != tempOtherMsgs.end()) {
                if((*itr).teamid == stoi(otherLeaderMsgs[i].id.substr(1)))
                    itr = tempOtherMsgs.erase(itr);
                else
                    itr++;
            }

            /* Was any non-team member detected? Cannot be closest to non-team member if none were detected */
            if(tempOtherMsgs.empty()) {
                isClosestToOther = false;
            }
            else {
                isClosestToOther = true;
            }

            /* Check the distance to a robot that's the closest to itself and not part of the currently considered team*/
            minNonTeamDistance = __FLT_MAX__;
            for(int j = 0 ; j < tempOtherMsgs.size(); j++) {
                Real dist = tempOtherMsgs[j].direction.Length();
                if(dist < minNonTeamDistance)
                    minNonTeamDistance = dist;
            }
            std::cout << "Dist to chain: " << minNonTeamDistance << std::endl;
            std::cout << "nonTeamMsgs: " << nonTeamMsgs.size() << std::endl;
            std::cout << "tempOtherMsgs: " << tempOtherMsgs.size() << std::endl;

            /* Check whether the leader or itself is closer to a non-team member */
            for(int j = 0; j < tempOtherMsgs.size(); j++) {
                CVector2 diff = otherLeaderMsgs[i].direction - tempOtherMsgs[j].direction;
                Real dist = diff.Length();
                std::cout << "Dist between LC: " << dist << std::endl;
                if(dist < minNonTeamDistance && otherLeaderMsgs[i].numOtherTeamSeen > 1) {  // Must be >1 to exclude itself, who is a chain
                    isClosestToOther = false;
                    break;
                }
            }
            if( !isClosestToOther )
                break;
        }
    }
}

/****************************************/
/****************************************/

CVector2 CFollower::GetLeaderFlockingVector() {
    CVector2 resVec = CVector2();
    if(leaderMsg.direction.Length() > 0.0f) {
        std::cout << leaderMsg.direction.Length() << std::endl;

        Real fPID = pid->calculate(m_sLeaderFlockingParams.TargetDistance,
                                   leaderMsg.direction.Length());
        std::cout << fPID << std::endl;

        resVec += CVector2(-fPID,
                           leaderMsg.direction.Angle());

        /* Clamp the length of the vector to the max speed */
        if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
            resVec.Normalize();
            resVec *= m_sWheelTurningParams.MaxSpeed;
        }
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CFollower::GetTeamFlockingVector() {
    CVector2 resVec = CVector2();
    int teammateSeen = teamMsgs.size();
    if(teammateSeen > 0) {

        for(int i = 0; i < teammateSeen; i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJones(teamMsgs[i].direction.Length());
            /* Sum to accumulator */
            resVec += CVector2(fLJ,
                               teamMsgs[i].direction.Angle());
        }
        /* Divide the accumulator by the number of blobs seen */
        resVec /= teammateSeen;
        /* Clamp the length of the vector to the max speed */
        if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
            resVec.Normalize();
            resVec *= m_sWheelTurningParams.MaxSpeed;
        }
    }
    return resVec;
}

/****************************************/
/****************************************/

CVector2 CFollower::GetRobotRepulsionVector() {
    CVector2 resVec = CVector2();
    int otherSeen = otherLeaderMsgs.size() + otherTeamMsgs.size() + chainMsgs.size();

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

        for(int i = 0; i < chainMsgs.size(); i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJonesRepulsion(chainMsgs[i].direction.Length());
            /* Sum to accumulator */
            resVec += CVector2(fLJ,
                               chainMsgs[i].direction.Angle());
            numRepulse++;
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

    /* Clamp the length of the vector to the max speed */
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
    CVector2 leaderForce = GetLeaderFlockingVector();
    CVector2 teamForce = GetTeamFlockingVector();
    CVector2 robotForce = GetRobotRepulsionVector();
    CVector2 obstacleForce = GetObstacleRepulsionVector();
    CVector2 sumForce = leaderForce + teamForce + robotForce + obstacleForce;

    /* DEBUGGING */
    if(this->GetId() == "F1") {
        std::cout << "leader: " << leaderForce.Length() << std::endl;
        std::cout << "team: " << teamForce.Length() << std::endl;
        std::cout << "other: " << robotForce.Length() << std::endl;
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
}

/****************************************/
/****************************************/

bool CFollower::IsWorking() {
    return performingTask;
}

/****************************************/
/****************************************/

/* Callback functions (Controllable events) */

void CFollower::Callback_Flock(void* data) {
    std::cout << "Action: Flock" <<std::endl;
    currentMoveType = MoveType::FLOCK;
}

void CFollower::Callback_Stop(void* data) {
    std::cout << "Action: Stop" <<std::endl;
    currentMoveType = MoveType::STOP;
}

void CFollower::Callback_StartTask(void* data) {
    std::cout << "Action: StartTask" <<std::endl;
    performingTask = true;
}

void CFollower::Callback_StopTask(void* data) {
    std::cout << "Action: StopTask" <<std::endl;
    performingTask = false;
}

void CFollower::Callback_JoinLeader(void* data) {
    std::cout << "Action: JoinLeader" <<std::endl;
    teamID = closestLeader;
    currentState = RobotState::FOLLOWER;
}

void CFollower::Callback_JoinChain(void* data) {
    std::cout << "Action: JoinChain" <<std::endl;
    teamID = 255;
    currentState = RobotState::CHAIN;
    joinChainTriggered = true; // Used to trigger TaskEnded
    connection_timer = 0; // Reset timer
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CFollower::Check_TaskEnded(void* data) {
    if(joinChainTriggered) {
        std::cout << "Event: " << 1 << " - taskEnded" << std::endl;
        joinChainTriggered = false;
        return 1;
    }
    std::cout << "Event: " << 0 << " - taskEnded" << std::endl;
    return 0;
}

unsigned char CFollower::Check_GetStart(void* data) {
    if(currentState == RobotState::FOLLOWER) {
        if(leaderMsg.taskSignal == 1) {
            std::cout << "Event: " << 1 << " - getStart" << std::endl;
            return 1;
        }
    }
    std::cout << "Event: " << 0 << " - getStart" << std::endl;
    return 0;
}

unsigned char CFollower::Check_GetStop(void* data) {
    if(currentState == RobotState::FOLLOWER) {
        if(leaderMsg.taskSignal == 0) {
            std::cout << "Event: " << 1 << " - getStop" << std::endl;
            return 1;
        }
    }
    std::cout << "Event: " << 0 << " - getStop" << std::endl;
    return 0;
}

unsigned char CFollower::Check_ChainNear(void* data) {
    if(currentState == RobotState::FOLLOWER) {
        if(minNonTeamDistance != __FLT_MAX__) {
            std::cout << "Event: " << (minNonTeamDistance < toChainThreshold) << " - chainNear" << std::endl;
            return minNonTeamDistance < toChainThreshold;
        } else {
            std::cout << "Event: " << 0 << " - chainNear" << std::endl; // No chain found. Cannot be near a chain
            return 0;
        }
    }
    else if(currentState == RobotState::CHAIN) {
        std::cout << "Event: " << 0 << " - chainNear" << std::endl;
        return 0;
    }
    std::cout << "Event: " << 1 << " - chainNear" << std::endl; // Robot is the chain
    return 1;
}

unsigned char CFollower::Check_ChainFar(void* data) {
    if(currentState == RobotState::FOLLOWER) {
        if(minNonTeamDistance != __FLT_MAX__) {
            std::cout << "Event: " << (minNonTeamDistance >= toChainThreshold) << " - chainFar" << std::endl;
            return minNonTeamDistance >= toChainThreshold;
        } else {
            std::cout << "Event: " << 1 << " - chainFar" << std::endl; // No chain found. Chain must be far
            return 1;
        }
    }
    std::cout << "Event: " << 0 << " - chainFar" << std::endl; // Robot is the chain
    return 0;
}

unsigned char CFollower::Check_LeaderNear(void* data) {
    if(currentState == RobotState::CHAIN) {
        for(int i = 0; i < otherLeaderMsgs.size(); i++) {
            if(otherLeaderMsgs[i].direction.Length() < toFollowThreshold) {
                std::cout << "Event: " << 1 << " - leaderNear(" << otherLeaderMsgs[i].teamid << ")" << std::endl;
                return 1;
            }
        }
    }
    std::cout << "Event: " << 0 << " - leaderNear" << std::endl;
    return 0;
}

unsigned char CFollower::Check_LeaderFar(void* data) {
    if(currentState == RobotState::CHAIN) {
        for(int i = 0; i < otherLeaderMsgs.size(); i++) {
            if(otherLeaderMsgs[i].direction.Length() < toFollowThreshold) {
                std::cout << "Event: " << 0 << " - leaderFar" << std::endl;
                return 0;
            }
        }
    }
    else if(currentState == RobotState::FOLLOWER) {
        std::cout << "Event: " << 0 << " - leaderFar" << std::endl;
        return 0;
    }
    std::cout << "Event: " << 1 << " - leaderFar" << std::endl;
    return 1;
}

unsigned char CFollower::Check_ClosestToChain(void* data) {
    std::cout << "Event: " << isClosestToOther << " - closestToChain" << std::endl; // Default output
    return isClosestToOther;
}

unsigned char CFollower::Check_NotClosestToChain(void* data) {
    std::cout << "Event: " << !isClosestToOther << " - notClosestToChain" << std::endl; // Default output
    return !isClosestToOther;
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
