/* Include the controller definition */
#include "leader.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <utility/team_color.h>
#include <algorithm>
#include <string.h>

//#define RType 10
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

CLeader::CLeader() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_pcLEDs(NULL),
    m_pcPosSens(NULL),
    m_bSelected(false),
    m_strUsername(""),
    m_bSignal(false),
    PIDHeading(NULL),
    nearRobot(false),
    sct(NULL) {}

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
        GetNodeAttribute(GetNode(t_node, "team_distance"), "separation_threshold", separationThres);
        /* Minimum duration the accept message will be sent for */
        GetNodeAttribute(GetNode(t_node, "timeout"), "send_message", sendDuration);
        /* Time to wait between sending each robot */
        GetNodeAttribute(GetNode(t_node, "timeout"), "send_robot_delay", sendRobotDelay);
        /* SCT Model */
        GetNodeAttribute(GetNode(t_node, "SCT"), "path", m_strSCTPath);
        GetNodeAttribute(GetNode(t_node, "timeout"), "request_delay", RequestDelay);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
    // std::cout << m_strSCTPath << std::endl;

    /* Get team ID from leader ID */
    teamID = stoi(GetId().substr(1));

    float timeInSeconds = sendRobotDelay / 10.0;
    sendRobotDelay = (size_t)ceil(timeInSeconds) * 10;

    /* Initialization */
    currentState = RobotState::LEADER;
    inputStart = true; // Make leader always send start signal for now
    inputStop = false;
    
    currentTaskId = "";
    currentTaskDemand = 0;
    currentInitTaskDemand = 0;

    //numOtherTaskRequire[RType] = {0};    // might need to change
    std::fill_n(numOtherTaskRequire, RType, 0);
    //numOtherFollower[RType] = {-1};      // might need to change
    std::fill_n(numOtherFollower, RType, -1);

    shareToTeam = "";
    initStepTimer = 0;
    robotLastSentTime = 0;
    request_start_time = 0;
    acceptID = "";

    lastSent = -1;
    lastBeatTime = 0;
    beatReceived = 0;
    beatSent = 0;

    //numRobotsToSend[RType] = {0};            //
    std::fill_n(numRobotsToSend, RType, 0);
    //numRobotsRemainingToSend[RType] = {0};   //
    std::fill_n(numRobotsRemainingToSend, RType, 0);
    //numRobotsToRequest[RType] = {0};         //
    std::fill_n(numRobotsToRequest, RType, 0);
    //numRobotsRequested[RType] = {0};       //
    std::fill_n(numRobotsRequested, RType, 0);
    isSendingRobots = false;
    switchCandidate = "";
    robotToSwitch = "";
    //switchCandidateType = 1;
    
    decremented = false;
    //robotsNeeded[RType] = {0};
    std::fill_n(robotsNeeded, RType, 0);
    requestSent = false;
    acknowledgeSent = false;
    request_required = false;
    // requestReceived = false;

    // TEMP: hard coded team to join (Assuming two teams)
    if(teamID == 1)
        teamToJoin = 2;
    else if(teamID == 2)
        teamToJoin = 1;

    /*
    * Init SCT Controller
    */
    sct = new SCT(m_strSCTPath);

    if( m_strSCTPath == "src/SCT_models/leader.yaml" ) {

        /* Without exchange */

        /* Register controllable events */
        sct->add_callback(this, sct->events["EV_start"],    &CLeader::Callback_Start,    NULL, NULL);
        sct->add_callback(this, sct->events["EV_stop"],     &CLeader::Callback_Stop,     NULL, NULL);
        // sct->add_callback(this, sct->events["EV_message"],  &CLeader::Callback_Message,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_respond"],  &CLeader::Callback_Respond,  NULL, NULL);

        /* Register uncontrollable events */
        // sct->add_callback(this, sct->events["EV__message"],      NULL, &CLeader::Check__Message,      NULL);
        // sct->add_callback(this, sct->events["EV__relay"],        NULL, &CLeader::Check__Relay,        NULL);
        sct->add_callback(this, sct->events["EV__requestL"],     NULL, &CLeader::Check__RequestL,     NULL);
        sct->add_callback(this, sct->events["EV_pressStart"],    NULL, &CLeader::Check_PressStart,    NULL);
        sct->add_callback(this, sct->events["EV_pressStop"],     NULL, &CLeader::Check_PressStop,     NULL);
        // sct->add_callback(this, sct->events["EV_inputMessage"],  NULL, &CLeader::Check_InputMessage,  NULL);

    } else if( m_strSCTPath == "src/SCT_models/leader_exchange.yaml" ) {

        /* With exchange */

        /* Register controllable events */
        sct->add_callback(this, sct->events["EV_start"],    &CLeader::Callback_Start,    NULL, NULL);
        sct->add_callback(this, sct->events["EV_stop"],     &CLeader::Callback_Stop,     NULL, NULL);
        sct->add_callback(this, sct->events["EV_message"],  &CLeader::Callback_Message,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_respond"],  &CLeader::Callback_Respond,  NULL, NULL);
        sct->add_callback(this, sct->events["EV_exchange"], &CLeader::Callback_Exchange, NULL, NULL);

        /* Register uncontrollable events */
        // sct->add_callback(this, sct->events["EV__message"],      NULL, &CLeader::Check__Message,      NULL);
        // sct->add_callback(this, sct->events["EV__relay"],        NULL, &CLeader::Check__Relay,        NULL);
        sct->add_callback(this, sct->events["EV__requestL"],     NULL, &CLeader::Check__RequestL,     NULL);
        sct->add_callback(this, sct->events["EV_pressStart"],    NULL, &CLeader::Check_PressStart,    NULL);
        sct->add_callback(this, sct->events["EV_pressStop"],     NULL, &CLeader::Check_PressStop,     NULL);
        sct->add_callback(this, sct->events["EV_inputMessage"],  NULL, &CLeader::Check_InputMessage,  NULL);
        sct->add_callback(this, sct->events["EV_inputExchange"], NULL, &CLeader::Check_InputExchange, NULL);
    
    } else {
        std::cout << "Unknown SCT file path: " << m_strSCTPath << std::endl;
    }

    /* Set LED color */
    // m_pcLEDs->SetAllColors(teamColor[teamID]);
    m_pcLEDs->SetAllColors(CColor::RED);

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
    delete sct;
}

/****************************************/
/****************************************/

void CLeader::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    cbyte_msg = CByteArray(Message::messageByteSize, 255);      // Setting the msg values to 255
    m_pcRABAct->SetData(cbyte_msg);

    /* Reset the incoming public events */
    // pub_events.clear();
    // pub_events[EV_b] = false;
}

/****************************************/
/****************************************/

void CLeader::ControlStep() {

    // std::cout << "\n---------- " << this->GetId() << " ----------" << std::endl;

    initStepTimer++;
    // std::cout << "TIME: " << initStepTimer << std::endl;

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/

    /* Clear messages received */
    teamMsgs.clear();
    connectorMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();
    travelerMsgs.clear();

    cmsgToSend.clear();
    rmsgToSend.clear();

    nearRobot = false;

    receivedMessage = false;
    receivedRelay = false;
    receivedRequest = false;
    inputMessage = false;

    lastControllableAction = "";

    robotToSwitch = "";

    // for(int i = 0; i < waypoints.size(); i++) {
    //     //std::cout << waypoints[i].GetX() << "," << waypoints[i].GetY() << std::endl;
    // }

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();
    
    /*------------------------*/
    /* Update sensor readings */
    /*------------------------*/
    Update();

    //std::cout << "Robot delay:" << sendRobotDelay << std::endl;

    /*--------------------*/
    /* Run SCT controller */
    /*--------------------*/
    //std::cout << "--- Supervisors ---" << std::endl;

    if(initStepTimer > 4)
        sct->run_step();    // Run the supervisor to get the next action
    
    // std::cout << "[" << this->GetId() << "] " << sct->get_current_state_string() << std::endl;
    std::cout << "[" << this->GetId() << "] Action: " << lastControllableAction << std::endl;

    /*-----------------------------*/
    /* Implement action to perform */
    /*-----------------------------*/

    /*---------*/
    /* Control */
    /*---------*/

    /* Store previous task demand */
    previousTaskDemand = currentTaskDemand;

    /* Set ConnectionMsg to send during this timestep */
    //std::cout << "resend size: " << cmsgToResend.size() << std::endl;
    for(auto it = cmsgToResend.begin(); it != cmsgToResend.end();) {
        if(it->first > 0) {
            cmsgToSend.push_back(it->second);
            it->first--; // Decrement timer
            ++it;
        } else {
            it = cmsgToResend.erase(it);
            acceptID = "";
        }
    }

    /* Set RelayMsg to send during this timestep */
    //std::cout << "resend size: " << rmsgToResend.size() << std::endl;
    for(auto it = rmsgToResend.begin(); it != rmsgToResend.end();) {
        if(it->first > 0) {
            rmsgToSend.push_back(it->second);
            it->first--; // Decrement timer
            ++it;
        } else {
            it = rmsgToResend.erase(it);
        }
    }

    /* Set LED */
    // if(m_bSignal)
    //     m_pcLEDs->SetAllColors(CColor::WHITE);
    // else {
    //     // m_pcLEDs->SetAllColors(teamColor[teamID]);
    //     m_pcLEDs->SetAllColors(CColor::RED);
    // }
    if(m_bSelected)
        m_pcLEDs->SetAllColors(CColor::RED);
    else
        m_pcLEDs->SetAllColors(CColor::BLACK);

    /* Set Motion */
    if(initStepTimer > 4) {

        /* Is the robot selected? */
        if(m_bSelected) {

            /* Follow the control vector */
            SetWheelSpeedsFromVectorEightDirections(m_cControl);
        }
        else {
            if( !nearRobot ) {
                /* Stop if other robots are too far from itself */
                m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            }
            else if( !waypoints.empty() ) {
                /* Check if it is near the waypoint */
                CVector3 pos3d = m_pcPosSens->GetReading().Position;
                CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
                Real dist = (waypoints.front() - pos2d).Length();
                //std::cout << "dist: " << dist << std::endl;

                /* If current task is completed, move to the next one */
                // Why not and used?
                if(dist > m_sWaypointTrackingParams.thresRange || currentTaskDemand == 0) {
                    
                    //std::cout << "[LOG] Moving to next task" << std::endl;

                    /* Calculate overall force applied to the robot */
                    CVector2 waypointForce = VectorToWaypoint();           // Attraction to waypoint
                    CVector2 robotForce    = GetRobotRepulsionVector();    // Repulsion from other robots
                    CVector2 obstacleForce = GetObstacleRepulsionVector(); // Repulsion from obstacles

                    CVector2 sumForce      = waypointForce + robotForce + obstacleForce;
                    //std::cout << "waypointForce: " << waypointForce << std::endl;
                    //std::cout << "robotForce: " << robotForce << std::endl;
                    //std::cout << "obstacleForce: " << obstacleForce << std::endl;
                    //std::cout << "sumForce: " << sumForce << std::endl;

                    SetWheelSpeedsFromVectorHoming(sumForce);
                } 
                else {
                    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
                }
            }
            else {
                m_pcWheels->SetLinearVelocity(0.0f, 0.0f);

                // FOR TRAINING

                /* Send the requested number of robots to the other leader */
                // if( !isSendingRobots ) {

                //     if(numRobotsRequested > 0) {
                //         SetRobotsToSend(numRobotsRequested);
                //         isSendingRobots = true;
                //         numRobotsRequested = 0; // reset
                //     }
                // }

                // if(numRobotsToSend == 0) {
                //     isSendingRobots = false;
                // }
            }
        }
    }
    
    /* Create new message to send */
    Message msg = Message(0);

    /* Set message content */
    msg.state = currentState;
    msg.ID = this->GetId();
    msg.teamID = teamID;
    msg.leaderSignal = m_bSignal;
    if( !robotToSwitch.empty() ) {
        msg.robotToSwitch = robotToSwitch;
        msg.teamToJoin = teamToJoin;
    }
    
    /* Hop Count */
    HopMsg hop;
    hop.count = 0;
    // Skip ID

    msg.hops[teamID] = hop;

    /* Connection Message */
    for(const auto& conMsg : cmsgToSend) {
        msg.cmsg.push_back(conMsg);
    }

    if( !shareToTeam.empty() ) {
        msg.shareToTeam = shareToTeam;
    }

    // Skip Teams Nearby

    /* Relay Message */
    for(const auto& relayMsg : rmsgToSend) {
        msg.rmsg.push_back(relayMsg);
    }

    /* Set ID of all connections to msg */
    std::vector<Message> allMsgs(teamMsgs);
    allMsgs.insert(std::end(allMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

    for(size_t i = 0; i < allMsgs.size(); i++) {
        msg.connections.push_back(allMsgs[i].ID);

        if(i >= 29)
            break;
    }
    msg.type = 0;

    //msg.Print(;)
    /* Convert message into CByteArray */
    cbyte_msg = msg.GetCByteArray();

    /*--------------*/
    /* Send message */
    /*--------------*/
    m_pcRABAct->SetData(cbyte_msg);

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

std::string CLeader::GetUsername() {
    return m_strUsername;
}

/****************************************/
/****************************************/

void CLeader::SetUsername(std::string username) {
    m_strUsername = username;
}

/****************************************/
/****************************************/

void CLeader::SetControlVector(const CVector2& c_control) {
   m_cControl = c_control;
}

/****************************************/
/****************************************/

void CLeader::SetSignal(const bool b_signal) {
    if(b_signal) {
        inputStart = true;
        inputStop = false;
    } else {
        inputStop = true;
        inputStart = false;
    }
}

/****************************************/
/****************************************/

void CLeader::SetRobotsToRequest(const UInt32 un_robots[]) {

    for(int i=0; i<RType; ++i){
        std::cout << "[" << this->GetId() << "] Received " << un_robots[i] << " robots of type " << i+1 << " to request from user" << std::endl;
        numRobotsToRequest[i] = un_robots[i];
    }
}

/****************************************/
/****************************************/

void CLeader::SetRobotsToSend(const UInt32 un_robots[]) {
    int currentTotalFollower = 0;
    for(int i=0; i<RType; ++i){
        currentTotalFollower += currentFollowerCount[i];
    }
    for(int i=0; i<RType; ++i){
        if(un_robots[i] !=0)
            std::cout << "[" << this->GetId() << "] Received request from user to send" << un_robots[i] << " robots of type " << i+1 << std::endl;
        
        
        if(currentTotalFollower <= 1) {
            std::cout << "{" << this->GetId() << "}[LOG] Cannot send if robots <= 1." << std::endl;
            break;
        } else if(currentFollowerCount[i] <= un_robots[i]) { // If robots to send exceed current team size, send all followers

            if(currentTotalFollower > currentFollowerCount[i])
                numRobotsToSend[i] = currentFollowerCount[i];
            else
                numRobotsToSend[i] = currentFollowerCount[i] - 1; // Keep one follower and send the rest
        } else {
            numRobotsToSend[i] = un_robots[i];
        }

        numRobotsRemainingToSend[i] = numRobotsToSend[i];
        currentTotalFollower -= numRobotsToSend[i];
    }
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

std::string CLeader::GetTaskId() {
    return currentTaskId;
}

/****************************************/
/****************************************/

void CLeader::SetTaskId(const std::string str_id) {
    currentTaskId = str_id;
}

/****************************************/
/****************************************/

UInt32 CLeader::GetTaskDemand() {
    return currentTaskDemand;
}

/****************************************/
/****************************************/

void CLeader::SetTaskDemand(const UInt32 un_demand) {
    currentTaskDemand = un_demand;
}

/****************************************/
/****************************************/

UInt32 CLeader::GetInitTaskDemand() {
    return currentInitTaskDemand;
}

/****************************************/
/****************************************/

void CLeader::SetInitTaskDemand(const UInt32 un_init_demand) {
    currentInitTaskDemand = un_init_demand;
}

/****************************************/
/****************************************/

UInt32* CLeader::GetMinimumCount() {
    static UInt32 arr[RType];
    for (int i=0; i<RType; ++i)
        arr[i] = robotsNeeded[i];
    return arr;
}

/****************************************/
/****************************************/

void CLeader::SetMinimumCount(UInt32 un_min[]) {
    for(int i=0;i<RType;++i)
        robotsNeeded[i] = un_min[i];
}

/****************************************/
/****************************************/

UInt32* CLeader::GetOtherMinimumCount() {
    static UInt32 arr[RType];
    for (int i=0; i<RType; ++i)
        arr[i] = numOtherTaskRequire[i];
    return arr;
}

/****************************************/
/****************************************/

UInt32* CLeader::GetFollowerCount() {
    static UInt32 arr[RType];
    for (int i=0; i<RType; ++i)
        arr[i] = currentFollowerCount[i];
    return arr;
}

/****************************************/
/****************************************/

void CLeader::SetFollowerCount(const UInt32 un_count[RType]) {
    for(int i =0; i<RType; ++i)
        currentFollowerCount[i] = un_count[i];
}

/****************************************/
/****************************************/

SInt32* CLeader::GetOtherFollowerCount() {
    static SInt32 arr[RType];
         for (int i=0; i<RType; ++i)
            arr[i] = numOtherFollower[i];
    return arr;
}

/****************************************/
/****************************************/

UInt8 CLeader::GetTeamID() {
    return teamID;
}

/****************************************/
/****************************************/

Real CLeader::GetLatestTimeSent() {
    return lastSent;
}

/****************************************/
/****************************************/

Real CLeader::GetLatestTimeReceived() {
    return lastBeatTime;
}

/****************************************/
/****************************************/

Real CLeader::GetTotalSent() {
    return beatSent;
}

/****************************************/
/****************************************/

Real CLeader::GetTotalReceived() {
    return beatReceived;
}

/****************************************/
/****************************************/

std::string CLeader::GetLastAction() {
    return lastControllableAction;
}

/****************************************/
/****************************************/

void CLeader::GetMessages() {
    
    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if( !tMsgs.empty() ) {
        for(int i = 0; i < tMsgs.size(); i++) {

            Message msg = Message(tMsgs[i]);

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
            } else if(msg.state == RobotState::TRAVELER) {
                msg.ID = 'F' + msg.ID;
                travelerMsgs.push_back(msg);
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

    CheckHeartBeat();

    /* If there are no followers in the team, cancel sending the */
    /* remaining number of robots                                */

    // for(int i=0; i<RType; ++i) {
    //     if(numRobotsToSend[i] > 0 && currentFollowerCount[i] == 0) {
    //         numRobotsToSend[i] = 0;
    //         numRobotsRemainingToSend[i] = 0;
    //     }
    // }

    int currentTotalFollower = 0;
    for(int i=0; i<RType; ++i){
        currentTotalFollower += currentFollowerCount[i];
    }
    if(currentTotalFollower == 0){                                                            
        for(int i=0; i<RType; ++i) {
            numRobotsToSend[i] = 0;
            numRobotsRemainingToSend[i] = 0;
        }
    }

    /* Only for simulated users */
    /* Check if task is completed or not to set signal to send */
    if( !m_bSelected ) {

        /* Simulated user signal */
        // bool signal = m_bSignal;

        if( !waypoints.empty() ) {

            /* Check if it is near the waypoint */
            CVector3 pos3d = m_pcPosSens->GetReading().Position;
            CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
            Real dist = (waypoints.front() - pos2d).Length();

            if(dist < m_sWaypointTrackingParams.thresRange) {

                /* Check if task is completed */
                if(currentTaskDemand == 0) {
                    // signal = false;
                    waypoints.pop(); // Delete waypoint from queue
                    requestSent = false; // Set to false since it has finished the task.
                    request_required = false;
                }/*  else
                    signal = true; */
            }
        }

        // if(signal && !m_bSignal)
        //     inputStart = true;
        // else if(!signal && m_bSignal)
        //     inputStop = true;   
    }
}

/****************************************/
//Check if atleast one follower is near the leader 
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
//Reply to request from follower to become a connector if there is no connector 
/****************************************/

void CLeader::ReplyToRequest() {

    /* Find the shortest distance to the other team */
    UInt8 minDist = 255;
    // std::string tempID;
    for(const auto& msg : teamMsgs) {
        if(msg.shareDist < minDist) {
            minDist = msg.shareDist;
            // std::cout << this->GetId() << ": dist is " << msg.shareDist << " from " << msg.ID << std::endl;
            // tempID = msg.ID;
        }
    }
    // std::cout << this->GetId() << ": separation received is " << minDist << " from " << tempID << std::endl;

    if(cmsgToResend.empty() && shareToTeam.empty() && minDist > separationThres) { // Check if it has not recently sent an accept message, whether there is already a connector or not, and whether there is a robot near the other team

        /* Upon receiving a request message, send an accept message to the follower with the smallest ID */
        for(const auto& teamMsg : teamMsgs) {
            for(const auto& cmsg : teamMsg.cmsg) {

                if(cmsg.to == this->GetId() && cmsg.type == 'R') {

                    receivedRequest = true;

                    /* Set the ID of the first follower request seen */
                    if(acceptID.empty()) {
                        acceptID = cmsg.from;
                        continue;
                    }
                    
                    UInt8 currentFID = stoi(acceptID.substr(1));
                    UInt8 newFID = stoi(cmsg.from.substr(1));

                    /* Send an accept message to the follower with the smallest ID */
                    if(newFID < currentFID)
                        acceptID = cmsg.from;
                }
            }
        }
    }
}

/****************************************/
//Get the first connector ID
/****************************************/

void CLeader::SetConnectorToRelay() {

    /* Check if a connector with a hop count = 1 to its current team is nearby */
    bool foundFirstConnector = false;
    for(const auto& msg : connectorMsgs) {
        auto hopInfo = msg.hops;

        if(hopInfo[teamID].count == 1) {

            /* Send info of connector found */
            shareToTeam = msg.ID;           // This ID is full 2 char - F1
            foundFirstConnector = true;
            break;
        }
    }

    /* If the first connector is not nearby, check which it should relay upstream */
    // When there is no connector - when teams are near each other
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
// HeartBeat msg received from the other leader
/****************************************/

void CLeader::CheckHeartBeat() {

    std::vector<Message> combinedMsgs(otherLeaderMsgs);
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(teamMsgs), std::end(teamMsgs));
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));

    std::string cand[RType];
    // std::string tail_connector[RType];
    // for(const auto& msg : connectorMsgs){
    //     auto hopInfo = msg.hops;
    //     if(hopInfo[teamID].count == 1)
    //         ;
    // }
    // //if (std::find(connections.begin(), connections.end(), msg.ID) != connections.end())
    // for(const auto& msg1 : teamMsgs) {
    //     UInt8 temp_type = msg1.type;
    //     for(const auto& msg2 : connectorMsgs){
    //         if(msg2.ID == shareToTeam){

    //         }

    //     }    
    //     //cand[temp_type-1] = 'F';
    //     cand[temp_type-1] = msg1.ID;
    //     //std::cout << "Type: " << temp_type << "    ID: " << msg.ID << "    cand: " << cand[temp_type-1] << std::endl;
    // }

    for(const auto& msg : teamMsgs) {
        UInt8 temp_type = msg.type;
        //cand[temp_type-1] = 'F';
        cand[temp_type-1] = msg.ID;
        //std::cout << "Type: " << temp_type << "    ID: " << msg.ID << "    cand: " << cand[temp_type-1] << std::endl;
    }

    

    for(const auto& msg : combinedMsgs) {
        //std::cout << "Leader ID: " << teamID << "     ID: " << msg.ID << "   Type: " << msg.type << std::endl;
        for(const auto& beat : msg.rmsg) {
            if(beat.from != this->GetId()) { 
                // if(this->GetId() == "L1")
                //     std::cerr << this->GetId() << " received " << lastBeatTime << "! (" << beatReceived << ")" << std::endl;

                if(beat.time > lastBeatTime) {
                    beatReceived++;
                    lastBeatTime = beat.time;
                    // std::cout << this->GetId() << " received " << lastBeatTime << "! (" << beatReceived << ")" << std::endl;
                    // if(this->GetId() == "L2")
                    //     std::cerr << this->GetId() << " received " << lastBeatTime << "! (" << beatReceived << ")" << std::endl;

                    if(msg.state == RobotState::LEADER)
                        receivedMessage = true;
                    else
                        receivedRelay = true;       //This is not being used

                    decremented = false;

                    /* Store message info from other leader */
                    for(int i=0; i<RType; ++i){
                        numOtherFollower[i] = beat.follower_num[i];
                        numOtherTaskRequire[i] = beat.task_min_num[i];
                    }

                    if(beat.type == 'R') {
                        // if( !requestReceived ) {
                        //     if(beat.robot_num != numPreviousRequest) {
                        //         numRobotsToSend += beat.robot_num - numPreviousRequest;
                        //         if(numRobotsToSend < 0)
                        //             numRobotsToSend = 0;
                        //         numPreviousRequest = beat.robot_num;
                        //         std::cout << this->GetId() << ": request from " << beat.from << " to send " << numRobotsToSend << " robots" << std::endl;

                        //     }
                        // }
                        for(int i=0; i<RType; ++i){
                            numRobotsRequested[i] = beat.robot_num[i];
                            if(numRobotsRequested[i]!= 0){
                        // std::cout << "{" << this->GetId() << "} [REQUEST] Received request from " << beat.from << " to send " << numRobotsRequested << " robots" << std::endl;
                            std::cout << "{" << this->GetId() << "}[REQUEST] Received request to send " << numRobotsRequested[i] << " robots of type:" << i+1 << std::endl;
                            }
                        }

                        // DEBUG
                        if( !m_bSelected ) {
                            int currentTotalFollower = 0;
                            for(int i=0; i<RType; ++i)
                                currentTotalFollower += currentFollowerCount[i];
                            for(int i=0; i<RType; ++i){
                                if(currentFollowerCount[i] <= numRobotsRequested[i]) {
                                    if(currentTotalFollower > currentFollowerCount[i])
                                        numRobotsToSend[i] = currentFollowerCount[i];
                                    else
                                        numRobotsToSend[i] = currentFollowerCount[i] - 1; // Keep one follower and send the rest
                                    //numRobotsToSend[i] = numRobotsRequested[i] - 1; // Keep one follower and send the rest
                                } else {
                                    numRobotsToSend[i] = numRobotsRequested[i];
                                }
                                numRobotsRemainingToSend[i] = numRobotsToSend[i];
                                currentTotalFollower -= numRobotsToSend[i];
                                // std::cout << "[LOG] " << numRobotsToSend << std::endl;
                            }
                        }

                    } else if(beat.type == 'A') {
                        // std::cout << this->GetId() << " Received Acknowledge from " << beat.from << " who is sending " << beat.robot_num << std::endl;
                        for(int i=0; i<RType; ++i)
                            std::cout << "{" << this->GetId() << "}[SEND] " << beat.robot_num[i] << " robots of type" << i+1 << "are heading this way!" << std::endl;
                    }

                    switchCandidate = ""; // Reset candidate follower to switch
                } 
                
                /* Set a follower that received the leader message from a non-team robot as a candidate to switch teams */
                // NEEDS TO BE CHANGED - next 4 lines commented and replaced with different logic
                // if( switchCandidate.empty() && !beat.firstFollower[].empty()) {
                //     switchCandidate = beat.firstFollower[];
                //     // std::cout << this->GetId() << ": first follower to receive was " << beat.firstFollower << std::endl;
                // }

                // for(int i=0; i<RType; ++i){
                //     if(numRobotsRemainingToSend[i]>0){
                //         switchCandidate = beat.firstFollower;   //To change
                //         switchCandidateType = i;
                //         break;
                //     }
                // }

                for(int i=0; i<RType; ++i){
                    if(numRobotsRemainingToSend[i]>0){
                        switchCandidate = cand[i];   //To change
                        switchCandidateType = i;
                        break;
                    }
                }
            }
        }
    }

    //std::cout << "lastBeatTime: " << lastBeatTime << std::endl;
    //std::cout << "beatReceived: " << beatReceived << std::endl;
}

/****************************************/
/****************************************/

CVector2 CLeader::VectorToWaypoint() {   
     /* Get current position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    //std::cout << "pos2d " << pos2d << std::endl;

    /* Calculate a normalized vector that points to the next waypoint */
    CVector2 cAccum = waypoints.front() - pos2d;

    //std::cout << "cAccum: " << cAccum << std::endl;
    //std::cout << "angle: " << cAccum.Angle() << std::endl;

    cAccum.Rotate((-cZAngle).SignedNormalize());
    
    //std::cout << "cAccum: " << cAccum << std::endl;
    //std::cout << "angle: " << cAccum.Angle() << std::endl;

    if(cAccum.Length() > 0.0f) {
        /* Make the vector as long as the max speed */
        cAccum.Normalize();
        cAccum *= m_sWheelTurningParams.MaxSpeed;
    }
    //std::cout << "cAccum: " << cAccum << std::endl;
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
        // //std::cout << "sensor " << i << ": " << vec.Length() << std::endl;
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

    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;

    if(c_heading.GetX() > 0) {
        /* Go straight */
        fSpeed1 = fBaseAngularWheelSpeed;
        fSpeed2 = fBaseAngularWheelSpeed;
    } else if(c_heading.GetX() < 0) {
        /* Go back */
        fSpeed1 = -fBaseAngularWheelSpeed;
        fSpeed2 = -fBaseAngularWheelSpeed;
    } else {
        /* Rotate */
        fSpeed1 = -fBaseAngularWheelSpeed;
        fSpeed2 = fBaseAngularWheelSpeed;
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

void CLeader::SetWheelSpeedsFromVectorEightDirections(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

    /* Wheel speeds based on current turning state */
    Real fLeftWheelSpeed = 0;
    Real fRightWheelSpeed = 0;

    if(c_heading.GetX() > 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed / 2;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed / 2;
        } else {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        }
    } else if(c_heading.GetX() < 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed / 2;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed / 2;
        } else {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        }
    } else if(c_heading.GetX() == 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        }
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
    //std::cout << fSpeed << std::endl;

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
    //std::cout << "[" << this->GetId() << "] ";
}

/****************************************/
/****************************************/

/* Callback functions (Controllable events) */

void CLeader::Callback_Start(void* data) {
    lastControllableAction = "start";
    m_bSignal = true;
    inputStart = false;
}

void CLeader::Callback_Stop(void* data) {
    lastControllableAction = "stop";
    m_bSignal = false;
    inputStop = false;
}

void CLeader::Callback_Message(void* data) {
    lastControllableAction = "message";

    /* Send a heart-beat message to the other leader every 10 timesteps */
    RelayMsg beat;
    beat.type = 'H';
    beat.from = this->GetId();
    beat.time = initStepTimer;
    for(int i=0; i<RType; ++i){
        beat.follower_num[i] = (UInt8)currentFollowerCount[i];
        beat.task_min_num[i] = (UInt8)robotsNeeded[i];
    }

    /* For every 10 timesteps, check if the demand is not decreasing to request robots from the other team */
    // if(exchangeUsed) {
    //     if(initStepTimer > 0 /* && initStepTimer % 10 == 0 */) {
    //         if(m_bSignal) { // Sending start signal to robots
    //             if(robotsNeeded - currentFollowerCount > 0) {
    //                 beat.type = 'R';
    //                 // beat.robot_num = 100; // TEMP: Large number to send all robots
    //                 beat.robot_num = robotsNeeded - currentFollowerCount + 1;
    //                 std::cout << this->GetId() << ": Sending request for " << beat.robot_num << " robots" << std::endl;
    //             }
    //         }
    //     }
    // }

    
    // DEBUG (Auto request)
    if(!request_required)
    for(int i=0; i<RType; ++i){
        if(robotsNeeded[i] - currentFollowerCount[i] > 0) { 
            request_start_time = initStepTimer;
            request_required = true;
            break;
        }
    }
    if( !m_bSelected && !requestSent && (initStepTimer - request_start_time)>RequestDelay) {
        for(int i=0; i<RType; ++i){
            //if(robotsNeeded - currentFollowerCount > 0 && !requestSent) {
            if(robotsNeeded[i] - currentFollowerCount[i] > 0) {    
                beat.type = 'R';
                beat.robot_num[i] = robotsNeeded[i] - currentFollowerCount[i];
                std::cout << "{" << this->GetId() << "}[REQUEST] Requesting " << beat.robot_num[i] << " robots of type " <<i+1 << std::endl;
                requestSent = true;
                //request_required = false;
            }
        }
    }

    // DEBUG (Auto send)        - IS THIS REQUIRED?
    // if( !m_bSelected ) {
    //     if(currentFollowerCount > 12 && !acknowledgeSent) {
    //         numRobotsToSend = currentFollowerCount - 7; // Send robots so that it has 7 followers left in team
    //         numRobotsRemainingToSend = numRobotsToSend;
    //     }
    // }

    /* User Signal */
    for(int i=0; i<RType; ++i){
        if(numRobotsToRequest[i]> 0) {
            beat.type = 'R';
            beat.robot_num[i] = numRobotsToRequest[i];
            if(beat.robot_num[i] != 0)
                std::cout << "{" << this->GetId() << "}[REQUEST] Requesting " << beat.robot_num[i] << " robots of type " <<i+1 << std::endl;
                
            // std::cout << "[" << this->GetId() << "] Requested for " << beat.robot_num << " robots" << std::endl;
            numRobotsToRequest[i] = 0;
        }
    }

    /* Acknowledge message */
    // std::cout << this->GetId() << " requested: " << numRobotsRequested << ", to send: " << numRobotsToSend << std::endl;
    // if(numRobotsRequested == numRobotsToSend && numRobotsToSend > 0) {
    //if(!acknowledgeSent && numRobotsToSend > 0) {
    bool temp_cond1 = false;
    bool temp_cond2 = true;
    for(int i=0; i<RType; ++i){
        if(numRobotsToSend[i]>0){
            temp_cond1 = true;
            break;
        }
    }
    for(int i=0; i<RType; ++i){
        if(numRobotsRemainingToSend[i]!=0){
            temp_cond2 = false;
            break;
        }
    }
    if(!acknowledgeSent && temp_cond1){
        for(int i=0; i<RType; ++i){
            beat.type = 'A';
            beat.robot_num[i] = numRobotsToSend[i];
            if(beat.robot_num[i] != 0)
                std::cout << "{" << this->GetId() << "}[SEND] Sending " << numRobotsToSend[i] << " robots of type " <<i+1 << std::endl;
                std::cout << "Sending " << switchCandidate << std::endl;
            acknowledgeSent = true;
        }
    } 
    //else if(numRobotsRemainingToSend == 0) {
    //else if((acknowledgeSent || numRobotsToSend == 0) && numRobotsRemainingToSend == 0) { //sujith made
    else if(temp_cond2){
        acknowledgeSent = false;
        for(int i=0; i<RType; ++i)
            numRobotsToSend[i] = 0;
        
    }
    // std::cout << this->GetId() << " remaining to send " << numRobotsRemainingToSend << std::endl;

    rmsgToResend.push_back({sendDuration,beat});
    lastSent = initStepTimer;

    if(beat.type == 'R')
        //lastControllableAction += "_" + std::to_string(beat.robot_num);
        lastControllableAction += "_request";

    beatSent++;
}

void CLeader::Callback_Respond(void* data) {
    lastControllableAction = "respond";

    ConnectionMsg response;

    if(this->GetId() == "L1") { // We assume leaders agree that only L1 accept requests from followers

        /* Upon receiving a request message, send an accept message to the follower with the smallest ID */
        response.type   = 'A';
        response.from   = this->GetId();
        response.to     = acceptID;
        response.toTeam = teamID;

    } else {
        
        response.type   = 'N';
        response.from   = this->GetId();
        response.to     = "F0"; // DUMMY ID
        response.toTeam = teamID;

    }

    cmsgToResend.push_back({sendDuration,response});
}

void CLeader::Callback_Exchange(void* data) {
    lastControllableAction = "exchange";

    if( !switchCandidate.empty() ) {
        
        /* Signal a follower to switch to the other team */
        if(!decremented) {
            if(initStepTimer - robotLastSentTime > sendRobotDelay) {
                robotToSwitch = switchCandidate;
                numRobotsRemainingToSend[switchCandidateType]--;
                decremented = true;
                robotLastSentTime = initStepTimer;
                std::cout << "[" << this->GetId() << "] Send " << robotToSwitch << " to team " << teamToJoin << std::endl;
            }
        }

    } else {
        std::cerr << "[" << this->GetId() << "] switchCandidate is empty" << std::endl;
    }
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CLeader::Check__Message(void* data) {     
    // std::cout << "Event: " << receivedMessage << " - _message" << std::endl;
    return receivedMessage;
}

unsigned char CLeader::Check__Relay(void* data) {       // NOT USED
    // std::cout << "Event: " << receivedRelay << " - _relay" << std::endl;
    return receivedRelay;
}

unsigned char CLeader::Check__RequestL(void* data) {
    // std::cout << "Event: " << receivedRequest << " - _requestL" << std::endl;
    return receivedRequest;
}

unsigned char CLeader::Check_PressStart(void* data) {
    // std::cout << "Event: " << pressStart << " - pressStart" << std::endl;
    return inputStart;
}

unsigned char CLeader::Check_PressStop(void* data) {
    // std::cout << "Event: " << pressStop << " - inputStop" << std::endl;
    return inputStop;
}

unsigned char CLeader::Check_InputMessage(void* data) {
    bool timeToSend = (initStepTimer > 0 && initStepTimer % 10 == 0);
    // std::cout << "Event: " << timeToSend << " - inputMessage" << std::endl;
    return timeToSend;
}

unsigned char CLeader::Check_InputExchange(void* data) {
    bool exchangeRobot = false;
    for(int i=0; i<RType; ++i)
        if(numRobotsRemainingToSend[i] > 0 && !switchCandidate.empty()){
            exchangeRobot = true;
            break;
        }

    if(exchangeRobot) {
        // std::cout << "Event: " << 1 << " - inputExchange" << std::endl;
        return 1;
    }
    // std::cout << "Event: " << 0 << " - inputExchange" << std::endl;
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
REGISTER_CONTROLLER(CLeader, "leader_controller")
