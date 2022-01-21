#include "manualcontrol_webvizuser_functions.h"

/****************************************/
/****************************************/

static const Real DIRECTION_VECTOR_FACTOR = 10.;

/****************************************/
/****************************************/

CManualControlWebvizUserFunctions::CManualControlWebvizUserFunctions() {
    m_pcExperimentLoopFunctions = static_cast<CExperimentLoopFunctions *>(
        &CSimulator::GetInstance().GetLoopFunctions());

    RegisterWebvizUserFunction<CManualControlWebvizUserFunctions, CEPuckLeaderEntity>(
        &CManualControlWebvizUserFunctions::sendLeaderData);

    RegisterWebvizUserFunction<CManualControlWebvizUserFunctions, CEPuckEntity>(
        &CManualControlWebvizUserFunctions::sendFollowerData);
}

/****************************************/
/****************************************/

CManualControlWebvizUserFunctions::~CManualControlWebvizUserFunctions() {}

/****************************************/
/****************************************/

void CManualControlWebvizUserFunctions::HandleCommandFromClient(const std::string& str_ip, 
                                                                nlohmann::json c_json_command) {

    if(c_json_command.empty())
        return;

    std::string client = c_json_command["client"];
    std::string username = c_json_command["username"];
    std::string target = c_json_command["robot"];
    nlohmann::json commands = c_json_command["commands"];

    /* Store the client's id and username if its the first time receiving it */
    if(m_pcClientPointerToId[str_ip].id == "") {
        m_pcClientPointerToId[str_ip].id = client;
    }
    if(username != "") {
        m_pcClientPointerToId[str_ip].username = username;
    }

    /* Apply commands from client */
    for (const auto& c_data : commands) {
        // std::cout << "value:" << c_data << std::endl;

        if(c_data["command"] == "move") {

            std::string direction = c_data["direction"];

            /* Get robot controller */
            CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
            for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                it != m_cEPuckLeaders.end();
                ++it) {

                /* Get handle to e-puck_leader entity and controller */
                CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                if(cController.GetId() == target) {
                    
                    /* Forward/backward direction factor (local robot X axis) */
                    SInt32 FBDirection = 0;
                    /* Left/right direction factor (local robot Y axis) */
                    SInt32 LRDirection = 0;
                    /* Calculate direction factor */
                    if(direction == "U") ++FBDirection;
                    if(direction == "D") --FBDirection;
                    if(direction == "L") ++LRDirection;
                    if(direction == "R") --LRDirection;
                    /* Calculate direction */
                    CVector2 cDir =
                        DIRECTION_VECTOR_FACTOR *
                        (CVector2(FBDirection, 0.0f) +
                        CVector2(0.0f, LRDirection));

                    /* Tell that e-puck that it is selected */
                    cController.Select();

                    /* Set direction */
                    cController.SetControlVector(cDir);

                    break;
                }
            }
        }
        else if(c_data["command"] == "task") {

            std::string signal = c_data["signal"];

            /* Get robot controller */
            CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
            for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                it != m_cEPuckLeaders.end();
                ++it) {

                /* Get handle to e-puck_leader entity and controller */
                CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                if(cController.GetId() == target) {
                    
                    /* Tell the e-puck to send a task signal */
                    if(signal == "start") {
                        std::cout << "[LOG]: (" << cController.GetId() << ") sending START task signal" << std::endl;
                        cController.SetSignal(true);
                    } else if(signal == "stop") {
                        std::cout << "[LOG]: (" << cController.GetId() << ") sending STOP task signal" << std::endl;
                        cController.SetSignal(false);
                    }

                    break;
                }
            }
        }
        else if(c_data["command"] == "request") {

            int num_robot = c_data["number"];

            std::cout << "Request received: " << num_robot << std::endl;

        }
        else if(c_data["command"] == "send") {

            int num_robot = c_data["number"];

            std::cout << "Send received: " << num_robot << std::endl;

            /* Get robot controller */
            CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
            for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                it != m_cEPuckLeaders.end();
                ++it) {

                /* Get handle to e-puck_leader entity and controller */
                CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                if(cController.GetId() == target) {
                    
                    /* Tell the e-puck to send its followers to the other team */
                    cController.SetRobotsToSend(num_robot);
                    break;
                }
            } 
        }
        else if(c_data["command"] == "select_leader") {

            // std::cout << "Select received (begin)" << std::endl;

            // for(const auto& [key, value] : m_pcClientRobotConnections) {
            //     std::cout << "robot: " << key << " - " << value.username << ", " << value.id << std::endl;
            // }

            // for(const auto& [key, value] : m_pcClientPointerToId) {
            //     std::cout << "pt: " << key << " - " << value.username << ", " << value.id << std::endl;
            // }

            /* Target robot is already controlled by a client */
            if(m_pcClientRobotConnections.count(target)) {
                ClientData clientInControl = m_pcClientRobotConnections[target];
                if(clientInControl.id == client) {
                    if(username != "") {
                        /* Update username */
                        CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
                        for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                            it != m_cEPuckLeaders.end();
                            ++it) {

                            /* Get handle to e-puck_leader entity and controller */
                            CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                            CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                            if(cController.GetId() == target) {
                                cController.SetUsername(username);
                                std::cout << "[LOG]: (" << target << ") connected to " << username << " (" << client << ")" << std::endl;
                                break;
                            }
                        }
                        m_pcClientRobotConnections[target].username = username;
                    }
                    continue;
                } 
                else if(clientInControl.id != "") { 
                    std::cout << "[ERR]: (" << target << ") is already being controlled by " 
                            << clientInControl.username << " (" << clientInControl.id << ")" << std::endl;
                    continue;
                }
            }
            
            /* Disconnect client from existing connections */
            for(auto& [key, value] : m_pcClientRobotConnections) {
                if(value.id == client) {

                    /* Deselect robot */
                    CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
                    for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                        it != m_cEPuckLeaders.end();
                        ++it) {

                        /* Get handle to e-puck_leader entity and controller */
                        CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                        CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                        if(cController.GetId() == key) {
                            cController.Deselect();
                            cController.SetUsername("");
                            cController.SetSignal(false);
                            break;
                        }
                    }

                    value = ClientData();
                    std::cout << "[LOG]: (" << key << ") released" << std::endl;
                }
            }

            if(target == "Select leader") {
                continue;
            }

            /* Connect client to leader */
            CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
            for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                it != m_cEPuckLeaders.end();
                ++it) {

                /* Get handle to e-puck_leader entity and controller */
                CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                if(cController.GetId() == target) {
                    cController.Select();
                    cController.SetUsername(username);

                    /* Update robot connection dict */
                    ClientData newClient;
                    newClient.id = client;
                    newClient.username = username;
                    m_pcClientRobotConnections[target] = newClient;

                    std::cout << "[LOG]: (" << target << ") connected to " << username << " (" << client << ")" << std::endl;
                }
            }
        }
    }
}

/****************************************/
/****************************************/

const nlohmann::json CManualControlWebvizUserFunctions::sendUserData() {
    nlohmann::json outJson;

    if(m_pcClientRobotConnections.empty()) {
        outJson["connections"] = nlohmann::json();
    } else {
        for(const auto& [key, value] : m_pcClientRobotConnections) {
            outJson["connections"][key]["id"] = value.id;
            outJson["connections"][key]["username"] = value.username;
        }
    }

    return outJson;
}

/****************************************/
/****************************************/

const nlohmann::json CManualControlWebvizUserFunctions::sendLeaderData(CEPuckLeaderEntity& robot) {
    nlohmann::json outJson;

    CLeader& cController = dynamic_cast<CLeader&>(robot.GetControllableEntity().GetController());
    
    /* Username of operator controlling the leader */
    outJson["username"] = cController.GetUsername();

    /* Number of followers */
    outJson["num_followers"] = cController.GetFollowerCount();

    /* Minimum number of robots needed for the current task */
    outJson["num_task_require"] = cController.GetMinimumCount();

    /* Task completion rate */

    /* Number of follower in the other team */
    outJson["num_other_followers"] = cController.GetOtherFollowerCount();

    /* Minimum number of robots needed for the other team's task */
    outJson["num_other_task_require"] = cController.GetOtherMinimumCount();

    return outJson;
}

/****************************************/
/****************************************/

const nlohmann::json CManualControlWebvizUserFunctions::sendFollowerData(CEPuckEntity& robot) {
    nlohmann::json outJson;

    CFollower& cController = dynamic_cast<CFollower&>(robot.GetControllableEntity().GetController());
    
    /* Robot's current state */
    switch(cController.GetRobotState()) {
        case RobotState::FOLLOWER: 
            outJson["state"] = "F";
            break;
        case RobotState::CONNECTOR: 
            outJson["state"] = "C";
            break;
        case RobotState::TRAVELER:
            outJson["state"] = "T";
            break;
        default:
            std::cerr << "[ERROR] Follower robot should not be a leader!" << std::endl;
            break;
    }

    return outJson;
}

/****************************************/
/****************************************/

void CManualControlWebvizUserFunctions::ClientConnected(std::string str_id) {
    std::cout << "Adding client " << str_id << std::endl;

    /* Create entry for connected client */
    m_pcClientPointerToId[str_id] = ClientData();
}

/****************************************/
/****************************************/

void CManualControlWebvizUserFunctions::ClientDisconnected(std::string str_id) {
    std::cout << "Disconnected " << str_id << std::endl;

    /* Release any robots that were selected by this client */
    for(auto& [key, value] : m_pcClientRobotConnections) {
        if(value.id == m_pcClientPointerToId[str_id].id) {

            /* Deselect robot */
            CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
            for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                it != m_cEPuckLeaders.end();
                ++it) {

                /* Get handle to e-puck_leader entity and controller */
                CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                if(cController.GetId() == key) {
                    cController.Deselect();
                    cController.SetUsername("");
                    cController.SetSignal(false);
                    break;
                }
            }

            value = ClientData();
            std::cout << "[LOG]: (" << key << ") released" << std::endl;
        }
    }

    /* Remove entry for connected client */
    m_pcClientPointerToId.erase(str_id);
}

/****************************************/
/****************************************/

REGISTER_WEBVIZ_USER_FUNCTIONS(
  CManualControlWebvizUserFunctions, "manualcontrol_webvizuser_functions")