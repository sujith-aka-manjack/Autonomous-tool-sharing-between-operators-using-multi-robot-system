/* Include the controller definition */
#include "follower.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

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
        GetNodeAttribute(GetNode(t_node, "team"), "chain_threshold", chainThreshold);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Set leader ID as team ID */
    if(leaderStr[0] != 'L') {
        THROW_ARGOSEXCEPTION("Non-leader name passed to follower. Leader name must be 'L<num>' format.");
    }
    teamID = stoi(leaderStr.substr(1));

    /*
    * Init SCT Controller
    */
    sct = new SCTProb();
    sct->add_callback(this, EV_flock,      &CFollower::Callback_Flock,      NULL, NULL);
    sct->add_callback(this, EV_stop,       &CFollower::Callback_Stop,       NULL, NULL);
    sct->add_callback(this, EV_joinLeader, &CFollower::Callback_JoinLeader, NULL, NULL);
    sct->add_callback(this, EV_joinChain,  &CFollower::Callback_JoinChain,  NULL, NULL);
    sct->add_callback(this, EV_wait,       &CFollower::Callback_Wait,       NULL, NULL);

    sct->add_callback(this, EV_leaderNear,  &CFollower::Callback_LeaderNear, &CFollower::Check_LeaderNear,  NULL);
    sct->add_callback(this, EV_leaderFar,   &CFollower::Callback_LeaderFar,  &CFollower::Check_LeaderFar,   NULL);
    sct->add_callback(this, EV_singleChain, NULL,                            &CFollower::Check_SingleChain, NULL);
    sct->add_callback(this, EV_multiChain,  NULL,                            &CFollower::Check_MultiChain,  NULL);

    Reset();
}

/****************************************/
/****************************************/

void CFollower::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    msg = CByteArray(10, 255);
    m_pcRABAct->SetData(msg);
    msg_index = 0;

}

/****************************************/
/****************************************/

void CFollower::ControlStep() {

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/

    /* Create new msg */
    msg = CByteArray(10, 255);
    msg_index = 0;

    /* Reset flocking variables */
    // leaderVec = CVector2();
    // teamVecs.clear();
    // otherVecs.clear();

    leaderMsg = Message();
    teamMsgs.clear();
    chainMsgs.clear();
    otherLeaderMsgs.clear();
    otherLeaderMsgs.clear();

    /* Reset chain information */
    connectingTargets.clear();
    identicalChain = 0;

    // chainVecs.clear();

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

    PrintName();
    sct->print_current_state();
    std::cout << ", R: " << static_cast<int>(currentState) << std::endl;

    /*-----------------------------*/
    /* Implement action to perform */
    /*-----------------------------*/

    /* Set current state in msg */
    msg[msg_index++] = static_cast<UInt8>(currentState);
    /* Set current team ID in msg */
    msg[msg_index++] = teamID;

    // Decide whether to communicate depending on current state (switch between follower and chain)
    switch(currentState) {
        case RobotState::FOLLOWER: {
            PrintName();
            std::cout << " FOLLOWER" << std::endl;
            break;
        }
        case RobotState::CHAIN: {
            PrintName();
            std::cout << " CHAIN" << std::endl;
            for(int i = 0; i < connectingTargets.size(); i++) {
                msg[msg_index++] = connectingTargets[i][0];    // First character of ID, 'L' or 'F'
                msg[msg_index++] = stoi(connectingTargets[i].substr(1));    // ID number
            }
            break;
        }
        case RobotState::LEADER: {
            PrintName();
            std::cout << "State is LEADER. Something went wrong." << std::endl;
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
        for(size_t i = 0; i < tMsgs.size(); ++i) {

            size_t index = 0;

            /* Get robot state */
            RobotState r_state = static_cast<RobotState>(tMsgs[i].Data[index++]);
            /* Get team ID */
            UInt8 r_team = tMsgs[i].Data[index++];

            // PrintName();
            // std::cout << "myteam=" << teamID;
            // std::cout << " rteam=" << r_team;
            // std::cout << " rstate=" << static_cast<UInt8>(r_state) << std::endl;

            /* Message from team */
            if(r_team == teamID) {
                /* Message from leader */
                if(r_state == RobotState::LEADER) {
                    /* Store position */
                    leaderMsg = Message();
                    leaderMsg.id = 'L' + std::to_string(r_team);
                    leaderMsg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);
                } 
                /* Message from follower */
                else if(r_state == RobotState::FOLLOWER) {
                    /* Store position */
                    Message msg = Message();
                    msg.id = 'F' + std::to_string(r_team);
                    msg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);
                    teamMsgs.push_back(msg);
                }
            } 
            /* Message from other team */
            else {
                /* Message from other leader */
                if(r_state == RobotState::LEADER) {
                    /* Store position (other leader is also part of the chain) */
                    Message msg = Message();
                    msg.id = 'L' + std::to_string(r_team);
                    msg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);
                    otherLeaderMsgs.push_back(msg);   
                }
                /* Message from other follower */
                else if(r_state == RobotState::FOLLOWER) {
                    /* Store position */
                    Message msg = Message();
                    msg.id = 'F' + std::to_string(r_team);
                    msg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);
                    otherTeamMsgs.push_back(msg); 
                }
            }

            /* Message from chain robot */
            if(r_state == RobotState::CHAIN) {
                std::cout << "Message from CHAIN" << std::endl;

                /* Store position */
                Message msg = Message();
                msg.id = 'F' + std::to_string(r_team);
                msg.direction = CVector2(tMsgs[i].Range, tMsgs[i].HorizontalBearing);

                /* Store which chain entities the other chain robot is connected to */
                size_t j = 0;
                while(tMsgs[i].Data[index] != 255) {
                    std::string chainID = (static_cast<RobotState>(tMsgs[i].Data[index++]) == RobotState::CHAIN) ? "F" : "L";
                    chainID += std::to_string(tMsgs[i].Data[index++]);
                    msg.connections[j++] = chainID;
                    // std::cout << msg.connections[j-1] << std::endl;
                }

                chainMsgs.push_back(msg);
            } 
        }
    }
}

/****************************************/
/****************************************/

void CFollower::UpdateSensors() {

    /* Check leader distance with chain (including other leader) */

    std::cout << "chainMsg=" << chainMsgs.size() << ", otherLMsg=" << otherLeaderMsgs.size() << std::endl;

    /* Combine messages from chain entities */
    std::vector<Message> combinedChainMsgs(chainMsgs);
    combinedChainMsgs.insert(std::end(combinedChainMsgs),
                             std::begin(otherLeaderMsgs),
                             std::end(otherLeaderMsgs));
    
    if(!combinedChainMsgs.empty()) {
        Real minLCDist = __FLT_MAX__;   // Minimum distance between the leader and the closest chain detected
        std::string chain1, chain2; // IDs of the two furthest chain entities
        Real maxDist1 = 0, maxDist2 = 0;
        for(int i = 0; i < combinedChainMsgs.size(); ++i) {
            /* For each chain position, check whether the distance between the leader and the chain
            exceeds the threshold. */
            CVector2 diff = leaderMsg.direction - combinedChainMsgs[i].direction;
            Real LCDist = diff.Length();
            if(LCDist < minLCDist)
                minLCDist = LCDist;

            if(currentState == RobotState::CHAIN) {
                /* Keep track of the two chain entities that are furthest from itself */
                Real dist = combinedChainMsgs[i].direction.Length();
                if(dist > maxDist1) {
                    chain2 = chain1;
                    maxDist2 = maxDist1;
                    chain1 = combinedChainMsgs[i].id;
                    maxDist1 = dist;
                } else if(dist > maxDist2) {
                    chain2 = combinedChainMsgs[i].id;
                    maxDist2 = dist;
                }
            }
        }

        LCDistance = minLCDist;
        PrintName();
        std::cout << LCDistance << std::endl;

        if(maxDist1 > 0) {
            connectingTargets.push_back(chain1);
            PrintName();
            std::cout << " ct1=" << connectingTargets[0] << ", ";
        }
        if(maxDist2 > 0) {
            connectingTargets.push_back(chain2);
            PrintName();
            std::cout << " ct2=" << connectingTargets[1];
        }
        std::cout << std::endl;
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

        for(int i = 0; i < teamMsgs.size(); ++i) {
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
    currentMoveType = MoveType::FLOCK;
}

void CFollower::Callback_Stop(void* data) {
    currentMoveType = MoveType::STOP;
}

void CFollower::Callback_JoinLeader(void* data) {
    currentState = RobotState::FOLLOWER;
}

void CFollower::Callback_JoinChain(void* data) {
    currentState = RobotState::CHAIN;
}

void CFollower::Callback_Wait(void* data) {}

void CFollower::Callback_LeaderNear(void* data) {
    m_pcLEDs->SetAllColors(CColor::BLACK);
}

void CFollower::Callback_LeaderFar(void* data) {
    m_pcLEDs->SetAllColors(CColor::RED);
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CFollower::Check_LeaderNear(void* data) {
    return LCDistance < chainThreshold;
}

unsigned char CFollower::Check_LeaderFar(void* data) {
    return LCDistance >= chainThreshold;
}

unsigned char CFollower::Check_SingleChain(void* data) {
    return identicalChain == 1;
}

unsigned char CFollower::Check_MultiChain(void* data) {
    return identicalChain > 1;
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
