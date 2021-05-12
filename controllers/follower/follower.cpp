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
    minChainDistance = __FLT_MAX__;
    isClosestToChain = true;
    connections.clear();

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

    // Decide whether to communicate depending on current state (switch between follower and chain)
    switch(currentState) {
        case RobotState::FOLLOWER: {
            std::cout << "State: FOLLOWER" << std::endl;
            m_pcLEDs->SetAllColors(CColor::BLACK);

            /* Set current team ID in msg */
            msg[msg_index++] = teamID;

            /* Set whether it has seen a chain */
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
            for(int i = 0; i < connections.size(); i++) {
                msg[msg_index++] = connections[i][0];    // First character of ID
                msg[msg_index++] = stoi(connections[i].substr(1));    // ID number

                if(i == 5)
                    break;
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
                    msg.hasSeenChain = tMsgs[i].Data[index++];
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
                    msg.hasSeenChain = tMsgs[i].Data[index++];
                    otherLeaderMsgs.push_back(msg);   
                }
                /* Message from other follower */
                else if(msg.state == RobotState::FOLLOWER) {
                    msg.id = 'F' + msg.id;
                    msg.hasSeenChain = tMsgs[i].Data[index++];
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
        std::vector<Message> combinedTeamMsgs(teamMsgs);
        if(leaderMsg.direction.Length() > 0.0f)
            combinedTeamMsgs.push_back(leaderMsg);

        Real minDist = __FLT_MAX__;
        for(int i = 0; i < combinedTeamMsgs.size(); i++) {
            for(int j = 0; j < combinedChainMsgs.size(); j++) {

                CVector2 diff = combinedTeamMsgs[i].direction - combinedChainMsgs[j].direction;
                Real dist = diff.Length();
                if(dist < minDist && combinedTeamMsgs[i].hasSeenChain) {
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
    else if(currentState == RobotState::CHAIN) {
        /* Check the distance to another chain that's the closest */
        for(int i = 0 ; i < chainMsgs.size(); i++) {
            Real dist = chainMsgs[i].direction.Length();
            if(dist < minChainDistance)
                minChainDistance = dist;
        }

        std::cout << "Dist to chain: " << minChainDistance << std::endl;
        
        /* Check if any leader is closer to the chain and has seen a chain */
        /* Event: closestToChain, notClosestToChain */
        Real minDist = __FLT_MAX__;
        for(int i = 0; i < otherLeaderMsgs.size(); i++) {
            for(int j = 0; j < chainMsgs.size(); j++) {

                CVector2 diff = otherLeaderMsgs[i].direction - chainMsgs[j].direction;
                Real dist = diff.Length();
                if(dist < minDist && otherLeaderMsgs[i].hasSeenChain) {
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

CVector2 CFollower::GetOtherRepulsionVector() {
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

void CFollower::Flock() {
    /* Calculate overall force applied to the robot */
    CVector2 leaderForce = GetLeaderFlockingVector();
    CVector2 teamForce = GetTeamFlockingVector();
    CVector2 otherForce = GetOtherRepulsionVector();
    CVector2 sumForce = leaderForce + teamForce + otherForce;

    /* DEBUGGING */
    if(this->GetId() == "F1") {
        std::cout << "leader: " << leaderForce.Length() << std::endl;
        std::cout << "team: " << teamForce.Length() << std::endl;
        std::cout << "other: " << otherForce.Length() << std::endl;
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

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CFollower::Check_ChainNear(void* data) {
    if(currentState == RobotState::FOLLOWER) {
        if(minChainDistance != __FLT_MAX__) {
            std::cout << "Event: " << (minChainDistance < toChainThreshold) << " - chainNear" << std::endl;
            return minChainDistance < toChainThreshold;
        } else {
            std::cout << "Event: " << 0 << " - chainNear" << std::endl; // No chain found. Cannot be near a chain
            return 0;
        }
    }
    std::cout << "Event: " << 1 << " - chainNear" << std::endl; // Robot is the chain
    return 1;
}

unsigned char CFollower::Check_ChainFar(void* data) {
    if(currentState == RobotState::FOLLOWER) {
        if(minChainDistance != __FLT_MAX__) {
            std::cout << "Event: " << (minChainDistance >= toChainThreshold) << " - chainFar" << std::endl;
            return minChainDistance >= toChainThreshold;
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
    }
    std::cout << "Event: " << 1 << " - leaderFar" << std::endl;
    return 1;
}

unsigned char CFollower::Check_ClosestToChain(void* data) {
    if(currentState == RobotState::FOLLOWER) {
        /* Check whether it is the closest to other chain entities (chain & other leaders) among nearby TEAMMATES */
        if( !chainMsgs.empty() || !otherLeaderMsgs.empty() ) {
            std::cout << "Event: " << (isClosestToChain == true) << " - closestToChain" << std::endl;
            return isClosestToChain == true;
        }
    }
    else if(currentState == RobotState::CHAIN) {
        /* Check whether it is the closest to other chains among nearby LEADERS */
        if( !chainMsgs.empty() ) {
            std::cout << "Event: " << (isClosestToChain == true) << " - closestToChain" << std::endl;
            return isClosestToChain == true;
        }
    }
    std::cout << "Event: " << 0 << " - closestToChain" << std::endl; // Default output
    return 0;
}

unsigned char CFollower::Check_NotClosestToChain(void* data) {
    if(currentState == RobotState::FOLLOWER) {
        /* Check whether it is NOT closest to other chain entities (chain & other leaders) among nearby TEAMMATES */
        if( !chainMsgs.empty() || !otherLeaderMsgs.empty() ) {
            std::cout << "Event: " << (isClosestToChain == false) << " - notClosestToChain" << std::endl;
            return isClosestToChain == false;
        }
    }
    else if(currentState == RobotState::CHAIN) {
        /* Check whether it is NOT closest to other chains among nearby LEADERS */
        if( !chainMsgs.empty() ) {
            std::cout << "Event: " << (isClosestToChain == false) << " - notClosestToChain" << std::endl;
            return isClosestToChain == false;
        }
    }
    std::cout << "Event: " << 1 << " - notClosestToChain" << std::endl; // Default output
    return 1;
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
