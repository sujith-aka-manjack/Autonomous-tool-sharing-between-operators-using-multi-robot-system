/* Include the controller definition */
#include "follower.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <algorithm>

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

CFollower::CFollower() :
    m_pcWheels(NULL),
    m_pcProximity(NULL){}

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
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"         );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing" );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing" );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds");

    std::string leaderStr;

    /*
    * Parse the config file
    */
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
        /* Flocking-related */
        m_sLeaderFlockingParams.Init(GetNode(t_node, "leader_flocking"));
        m_sTeamFlockingParams.Init(GetNode(t_node, "team_flocking"));
        /* Initial team ID */
        GetNodeAttribute(GetNode(t_node, "team"), "leader", leaderStr);
        /* Chain formation threshold */
        GetNodeAttribute(GetNode(t_node, "team"), "to_chain_threshold", toChainThreshold);
        GetNodeAttribute(GetNode(t_node, "team"), "to_follow_threshold", toFollowThreshold);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Set leader ID as team ID */
    if(leaderStr[0] != 'L') {
        THROW_ARGOSEXCEPTION("Non-leader name passed to follower. Leader name must be 'L<num>' format.");
    }
    teamID = stoi(leaderStr.substr(1));

    /* Set initial state to follower */
    currentState = RobotState::FOLLOWER;

    /*
    * Init SCT Controller
    */
    sct = new SCTProb();
    sct->add_callback(this, EV_flock,      &CFollower::Callback_Flock,      NULL, NULL);
    sct->add_callback(this, EV_stop,       &CFollower::Callback_Stop,       NULL, NULL);
    sct->add_callback(this, EV_joinLeader, &CFollower::Callback_JoinLeader, NULL, NULL);
    sct->add_callback(this, EV_joinChain,  &CFollower::Callback_JoinChain,  NULL, NULL);
    sct->add_callback(this, EV_wait,       &CFollower::Callback_Wait,       NULL, NULL);

    sct->add_callback(this, EV_chainNear,         NULL, &CFollower::Check_ChainNear,         NULL);
    sct->add_callback(this, EV_chainFar,          NULL, &CFollower::Check_ChainFar,          NULL);
    sct->add_callback(this, EV_closestToChain,    NULL, &CFollower::Check_ClosestToChain,    NULL);
    sct->add_callback(this, EV_notClosestToChain, NULL, &CFollower::Check_NotClosestToChain, NULL);
    sct->add_callback(this, EV_leaderNear,        NULL, &CFollower::Check_LeaderNear,        NULL);
    sct->add_callback(this, EV_leaderFar,         NULL, &CFollower::Check_LeaderFar,         NULL);
    sct->add_callback(this, EV_singleChain,       NULL, &CFollower::Check_SingleChain,       NULL);
    sct->add_callback(this, EV_multiChain,        NULL, &CFollower::Check_MultiChain,        NULL);

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

    /* Reset flocking variables */
    leaderMsg = Message();
    teamMsgs.clear();
    chainMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();

    /* Reset sensor reading results */
    minChainDistance = __FLT_MAX__;
    isClosestToChain = true;
    connectingTargets.clear();
    identicalChain = 0;

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    ReceiveMsg();

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

    // Decide whether to communicate depending on current state (switch between follower and chain)
    switch(currentState) {
        case RobotState::FOLLOWER: {
            std::cout << "State: FOLLOWER" << std::endl;
            m_pcLEDs->SetAllColors(CColor::BLACK);

            /* Set current team ID in msg */
            msg[msg_index++] = teamID;

            /* Set whether chain has been seen */
            if( !chainMsgs.empty() || !otherLeaderMsgs.empty() )
                msg[msg_index++] = 1;
            else
                msg[msg_index++] = 0;
            break;
        }
        case RobotState::CHAIN: {
            std::cout << "State: CHAIN" << std::endl;
            m_pcLEDs->SetAllColors(CColor::RED);

            /* Set current team ID in msg */
            teamID = 255;
            msg[msg_index++] = teamID;

            /* Set connected chain entity info to message (up to 6) */
            for(int i = 0; i < connectingTargets.size(); i++) {
                msg[msg_index++] = connectingTargets[i][0];    // First character of ID
                msg[msg_index++] = stoi(connectingTargets[i].substr(1));    // ID number
            }
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

void CFollower::ReceiveMsg() {

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
                /* Message from leader */
                if(msg.state == RobotState::LEADER) {
                    msg.id = 'L' + msg.id;
                    leaderMsg = msg;
                } 
                /* Message from follower */
                else if(msg.state == RobotState::FOLLOWER) {
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

void CFollower::UpdateSensors() {

    /* Check leader distance with chain (including other leader) */

    std::cout << "chainMsg  = " << chainMsgs.size() << std::endl;
    std::cout << "otherLMsg = " << otherLeaderMsgs.size() << std::endl;

    /* Combine messages received from chain entities (chain robot and other leaders) */
    std::vector<Message> combinedChainMsgs(chainMsgs);
    combinedChainMsgs.insert(std::end(combinedChainMsgs),
                             std::begin(otherLeaderMsgs),
                             std::end(otherLeaderMsgs));

    
    if(currentState == RobotState::FOLLOWER) {

        /* Check the distance to the closest chain entity */
        /* Event: chainNear, chainFar */
        for(int i = 0 ; i < combinedChainMsgs.size(); i++) {
            Real dist = combinedChainMsgs[i].direction.Length();
            if(dist < minChainDistance)
                minChainDistance = dist;
        }

        std::cout << "Dist to chain: " << minChainDistance << std::endl;

        /* Check if any team member is closer to the chain and has seen a chain */
        /* Event: closestToChain, notClosestToChain */
        Real minDist = __FLT_MAX__;
        for(int i = 0; i < teamMsgs.size(); i++) {
            for(int j = 0; j < combinedChainMsgs.size(); j++) {

                CVector2 diff = teamMsgs[i].direction - combinedChainMsgs[j].direction;
                Real dist = diff.Length();
                if(dist < minDist && teamMsgs[i].hasSeenChain) {
                    minDist = dist;
                    if(minDist < minChainDistance) {
                        isClosestToChain = false;
                        break;
                    }
                }
            }
            if( !isClosestToChain )
                break;
        }
    }

    /* Check the connected chain entities this robot is currently connected to */
    /* Count how many other robots connect to the same robots */
    /* Event: singleChain, multiChain */
    if(currentState == RobotState::CHAIN) {

        /* Add messages from follower robots */
        combinedChainMsgs.insert(std::end(combinedChainMsgs),
                                 std::begin(otherTeamMsgs),
                                 std::end(otherTeamMsgs));

        /* Extract ids from combinedMsgs */
        std::vector<std::string> connections;
        for(int i = 0; i < combinedChainMsgs.size(); i++) {
            if(combinedChainMsgs[i].state == RobotState::FOLLOWER) {
                connections.push_back("L" + combinedChainMsgs[i].teamid); // For follower robots, use their team id instead
            } else {
                connections.push_back(combinedChainMsgs[i].id);
            } 
        }

        /* Sort and remove duplicate ids */
        std::sort( connections.begin(), connections.end() );
        connections.erase( std::unique( connections.begin(), connections.end() ), connections.end() );

        /* Find the number of other chain robots that covers all of this robot's connections */
        for(int i = 0; i < chainMsgs.size(); i++) {
            std::vector<std::string> own(connections);                 /* This  robot's connections */
            std::vector<std::string> other = chainMsgs[i].connections; /* Other robot's connections */

            /* Delete each other's id from both connections */
            own.erase(std::remove(own.begin(), own.end(), chainMsgs[i].id), own.end());
            other.erase(std::remove(other.begin(), other.end(), this->GetId()), other.end());

            size_t index = 0;
            bool reachedEnd = false;
            for(int j = 0; j < other.size(); j++) {

                // compare own connection with other
                int result = own[index].compare(other[j]);

                if(result < 0) {
                    // Own connection is before other. The other robot does not cover this robot's connections.
                    break;
                } else if(result == 0) {
                    // Id match. Move on to next
                    index++;
                }

                if(index == own.size())
                    reachedEnd = true;
            }
            /* If reached to the end, increment identical. There is a chain robot that covers all of this robot's connections */
            if(reachedEnd)
                identicalChain++;
        }
    }
}

/****************************************/
/****************************************/

CVector2 CFollower::GetLeaderFlockingVector() {
    CVector2 resVec;
    if(leaderMsg.direction.Length() > 0.0f) {
        /*
        * Take the blob distance and angle
        * With the distance, calculate the Lennard-Jones interaction force
        * Form a 2D vector with the interaction force and the angle
        * Sum such vector to the accumulator
        */
        /* Calculate LJ */
        Real fLJ = m_sLeaderFlockingParams.GeneralizedLennardJones(leaderMsg.direction.Length());
        /* Sum to accumulator */
        resVec += CVector2(fLJ,
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
    CVector2 resVec;
    Real teammateSeen = teamMsgs.size();
    if(teammateSeen > 0) {

        for(int i = 0; i < teamMsgs.size(); i++) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJones(teamMsgs[i].direction.Length());
            /* Sum to accumulator */
            resVec += CVector2(fLJ,
                               teamMsgs[i].direction.Angle());
        }
        /* Divide the accumulator by the number of blobs seen */
        resVec /= teamMsgs.size();
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

CVector2 CFollower::GetOtherRepulsionVector() {
    
    return CVector2();
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

void CFollower::Flock() {
    /* Calculate overall force applied to the robot */
    CVector2 leaderForce = GetLeaderFlockingVector();
    CVector2 teamForce = GetTeamFlockingVector();
    CVector2 sumForce = leaderForce + teamForce;

    /* DEBUGGING */
    // if(this->GetId() == "F1") {
    //     std::cout << "leader: " << leaderForce.Length() << std::endl;
    //     std::cout << "team: " << teamForce.Length() << std::endl;
    //     std::cout << "sum: " << sumForce.Length() << std::endl;
    // }

    /* Set Wheel Speed */
    if(sumForce.Length() > 0.0f)
        SetWheelSpeedsFromVector(sumForce);
    else
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

/****************************************/
/****************************************/

void CFollower::PrintName() {
    std::cout << "[" << this->GetId() << "] ";
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

void CFollower::Callback_JoinLeader(void* data) {
    std::cout << "Action: JoinLeader" <<std::endl;
    currentState = RobotState::FOLLOWER;
}

void CFollower::Callback_JoinChain(void* data) {
    std::cout << "Action: JoinChain" <<std::endl;
    currentState = RobotState::CHAIN;
}

void CFollower::Callback_Wait(void* data) {
    std::cout << "Action: Wait" <<std::endl;
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CFollower::Check_ChainNear(void* data) {
    if(currentState == RobotState::FOLLOWER && minChainDistance != __FLT_MAX__) {
        std::cout << "Event: " << (minChainDistance < toChainThreshold) << " - chainNear" << std::endl;
        return minChainDistance < toChainThreshold;
    }
    std::cout << "Event: " << 0 << " - chainNear" << std::endl;
    return 0;
}

unsigned char CFollower::Check_ChainFar(void* data) {
    if(currentState == RobotState::FOLLOWER && minChainDistance != __FLT_MAX__) {
        std::cout << "Event: " << (minChainDistance >= toChainThreshold) << " - chainFar" << std::endl;
        return minChainDistance >= toChainThreshold;
    }
    std::cout << "Event: " << 0 << " - chainFar" << std::endl;
    return 0;
}

unsigned char CFollower::Check_ClosestToChain(void* data) {
    if(currentState == RobotState::FOLLOWER && (!chainMsgs.empty() || !otherLeaderMsgs.empty())) {
        std::cout << "Event: " << (isClosestToChain == true) << " - closestToChain" << std::endl;
        return isClosestToChain == true;
    }
    std::cout << "Event: " << 0 << " - closestToChain" << std::endl;
    return 0;
}

unsigned char CFollower::Check_NotClosestToChain(void* data) {
    if(currentState == RobotState::FOLLOWER && (!chainMsgs.empty() || !otherLeaderMsgs.empty())) {
        std::cout << "Event: " << (isClosestToChain == false) << " - notClosestToChain" << std::endl;
        return isClosestToChain == false;
    }
    std::cout << "Event: " << 0 << " - notClosestToChain" << std::endl;
    return 0;
}

unsigned char CFollower::Check_LeaderNear(void* data) {
    if(currentState == RobotState::CHAIN) {
        for(int i = 0; i < otherLeaderMsgs.size(); i++) {
            if(otherLeaderMsgs[i].direction.Length() < toFollowThreshold) {
                teamID = otherLeaderMsgs[i].teamid;
                std::cout << "Event: " << 1 << " - leaderNear(" << teamID << ")" << std::endl;
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
        std::cout << "Event: " << 1 << " - leaderFar" << std::endl;
        return 1;
    }
    std::cout << "Event: " << 0 << " - leaderFar" << std::endl;
    return 0;
}

unsigned char CFollower::Check_SingleChain(void* data) {
    if(currentState == RobotState::CHAIN) {
        std::cout << "Event: " << (identicalChain == 0) << " - SingleChain" << std::endl;
        return identicalChain == 0;
    }
    std::cout << "Event: " << 0 << " - SingleChain" << std::endl;
    return 0;
}

unsigned char CFollower::Check_MultiChain(void* data) {
    if(currentState == RobotState::CHAIN) {
        std::cout << "Event: " << (identicalChain > 0) << " - MultiChain" << std::endl;
        return identicalChain > 0;
    }
    std::cout << "Event: " << 0 << " - MultiChain" << std::endl;
    return 0;
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
