/* Include the controller definition */
#include "follower2.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

void CFollower2::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

void CFollower2::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
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
Real CFollower2::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CFollower2::CFollower2() :
    m_pcWheels(NULL),
    m_pcProximity(NULL){}

/****************************************/
/****************************************/

CFollower2::~CFollower2() {
    delete sct;
}

/****************************************/
/****************************************/

void CFollower2::Init(TConfigurationNode& t_node) {

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

    std::stringstream ss(leaderStr.substr(1));
    ss >> teamID;

    /*
    * Init SCT Controller
    */
    sct = new SCTProb();
    sct->add_callback(this, EV_flock, &CFollower2::Callback_Flock, NULL, NULL);
    // sct->add_callback(this, EV_stop, &CFollower2::Callback_Stop, NULL, NULL);
    // sct->add_callback(this, EV_joinLeader, &CFollower2::Callback_JoinLeader, NULL, NULL);
    // sct->add_callback(this, EV_joinChain, &CFollower2::Callback_JoinChain, NULL, NULL);
    // sct->add_callback(this, EV_wait, &CFollower2::Callback_Wait, NULL, NULL);

    // sct->add_callback(this, EV_leaderNear, NULL, &CFollower2::Check_LeaderNear, NULL);
    // sct->add_callback(this, EV_leaderFar, NULL, &CFollower2::Check_LeaderFar, NULL);
    // sct->add_callback(this, EV_singleChain, NULL, &CFollower2::Check_SingleChain, NULL);
    // sct->add_callback(this, EV_multiChain, NULL, &CFollower2::Check_MultiChain, NULL);

    Reset();
}

/****************************************/
/****************************************/

void CFollower2::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    msg = CByteArray(10, 255);
    m_pcRABAct->SetData(msg);
    msg_index = 0;

}

/****************************************/
/****************************************/

void CFollower2::ControlStep() {

    /*** MESSAGE INIT ***/
    msg = CByteArray(10, 255);
    msg_index = 0;
    /* Set its state in msg */
    msg[msg_index++] = FOLLOWER;
    /* Set team ID in msg */
    msg[msg_index++] = teamID;

    /* Reset flocking variables */
    leaderVec = CVector2();
    teamVecs.clear();
    otherVecs.clear();

    /* Reset chain formation variables */
    chainVecs.clear();

    /* Process messages */
    GetMessages();

    /*** FLOCK ***/

    /* Calculate overall force applied to the robot */
    CVector2 leaderForce = GetLeaderFlockingVector();
    CVector2 teamForce = GetTeamFlockingVector();
    CVector2 sumForce = leaderForce + teamForce;

    /*** CHAIN ***/

    /* Check leader distance with chain (including other leader) */
    CheckJoinChain();

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

    /* Set message to send */
    m_pcRABAct->SetData(msg);

}

/****************************************/
/****************************************/

void CFollower2::GetMessages() {

    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if(! tMsgs.empty()) {

        for(size_t i = 0; i < tMsgs.size(); ++i) {

            size_t index = 0;

            /* Get robot state */
            size_t r_state = tMsgs[i].Data[index++];
            /* Get team ID */
            size_t r_team = tMsgs[i].Data[index++];

            if(r_team == teamID) {
                if(r_state == LEADER) {
                    leaderVec = CVector2(tMsgs[i].Range,
                                         tMsgs[i].HorizontalBearing);
                } else if(r_state == FOLLOWER) {
                    teamVecs.push_back(CVector2(tMsgs[i].Range, 
                                                tMsgs[i].HorizontalBearing));
                }
            } else if(r_state == CHAIN || r_state == LEADER) {
                /* Store chain positions */
                chainVecs.push_back(CVector2(tMsgs[i].Range, 
                                             tMsgs[i].HorizontalBearing));
                
                //AVOID

            } else {
                // AVOID
            }
        }
    }

    // for(auto itr = pub_events.begin(); itr != pub_events.end(); ++itr) {
    //     std::cout << "key = " << itr->first           // print key
    //               << ", val = " << itr->second << "\n";    // print value
    // }
}

/****************************************/
/****************************************/

void CFollower2::UpdateSensors() {}

/****************************************/
/****************************************/

CVector2 CFollower2::GetLeaderFlockingVector() {
    CVector2 resVec;
    if(leaderVec.Length() > 0.0f) {
        /*
        * Take the blob distance and angle
        * With the distance, calculate the Lennard-Jones interaction force
        * Form a 2D vector with the interaction force and the angle
        * Sum such vector to the accumulator
        */
        /* Calculate LJ */
        Real fLJ = m_sLeaderFlockingParams.GeneralizedLennardJones(leaderVec.Length());
        /* Sum to accumulator */
        resVec += CVector2(fLJ,
                           leaderVec.Angle());
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

CVector2 CFollower2::GetTeamFlockingVector() {
    CVector2 resVec;
    Real teammateSeen = teamVecs.size();
    if(teammateSeen > 0) {

        for(int i = 0; i < teamVecs.size(); ++i) {
            /* Calculate LJ */
            Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJones(teamVecs[i].Length());
            /* Sum to accumulator */
            resVec += CVector2(fLJ,
                               teamVecs[i].Angle());
        }
        /* Divide the accumulator by the number of blobs seen */
        resVec /= teamVecs.size();
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

CVector2 CFollower2::GetOtherRepulsionVector() {
    
    return CVector2();
}

/****************************************/
/****************************************/

void CFollower2::SetWheelSpeedsFromVector(const CVector2& c_heading) {
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

void CFollower2::CheckJoinChain() {
    
    /* For each chain position, check whether the distance between the leader and the chain
    exceeds the threshold. */
    if(!chainVecs.empty()) {
        Real minDistLeaderChain = 10000000; // Minimum distance between the leader and the closest chain detected (init 100km)

        for(int i = 0; i < chainVecs.size(); ++i) {
            CVector2 diff = leaderVec - chainVecs[i];
            Real dist = diff.Length();
            if(dist < minDistLeaderChain)
                minDistLeaderChain = dist;
        
        }

        PrintName();
        std::cout << "hey" << std::endl;
        std::cout << minDistLeaderChain << std::endl;

        /* If closest chain is far from leader, become a chain robot */
        if(minDistLeaderChain > chainThreshold) {
            std::cout << "FORM CHAIN" << std::endl;
            m_pcLEDs->SetAllColors(CColor::RED);
        } else {
            m_pcLEDs->SetAllColors(CColor::BLACK);
        }
    }
}

/****************************************/
/****************************************/

void CFollower2::PrintName() {
    std::cout << "[" << this->GetId() << "] ";
}

/****************************************/
/****************************************/

void CFollower2::Callback_Flock(void* data) {

}

// void CFollower2::Callback_Stop(void* data) {
    
// }

// void CFollower2::Callback_JoinLeader(void* data) {
    
// }

// void CFollower2::Callback_JoinChain(void* data) {
    
// }

// void CFollower2::Callback_Wait(void* data) {}

// /****************************************/
// /****************************************/

// unsigned char CFollower2::Check_LeaderNear(void* data) {
//     return 0;
// }

// unsigned char CFollower2::Check_LeaderFar(void* data) {
//     return 0;
// }

// unsigned char CFollower2::Check_SingleChain(void* data) {
//     return 0;
// }

// unsigned char CFollower2::Check_MultiChain(void* data) {
//     return 0;
// }

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
REGISTER_CONTROLLER(CFollower2, "follower2_controller")
