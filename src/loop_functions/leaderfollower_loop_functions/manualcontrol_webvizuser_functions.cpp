#include "manualcontrol_webvizuser_functions.h"

/****************************************/
/****************************************/

static const Real DIRECTION_VECTOR_FACTOR = 10.;

/****************************************/
/****************************************/

CManualControlWebvizUserFunctions::CManualControlWebvizUserFunctions() {
    m_pcExperimentLoopFunctions = static_cast<CExperimentLoopFunctions *>(
        &CSimulator::GetInstance().GetLoopFunctions());

    m_bLogging = m_pcExperimentLoopFunctions->IsLogging();
    if(m_bLogging) {
        m_strCommandFilePath = m_pcExperimentLoopFunctions->GetCommandFilePath();

        /* Write to file */
        m_cOutput.open(m_strCommandFilePath.c_str(), std::ios_base::app);
        m_cOutput << "TIME,USER,ROBOT,COMMAND,VALUE"; // Header
        m_cOutput.close();
    }

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

    /* Current timestep */
    UInt32 timestep = m_pcExperimentLoopFunctions->GetSpace().GetSimulationClock();

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
                    for(std::string::size_type i = 0; i < direction.size(); i++) {
                        if(direction[i] == 'U') ++FBDirection;
                        if(direction[i] == 'D') --FBDirection;
                        if(FBDirection < 0) {
                            /* Invert LR direction when moving backwards */
                            if(direction[i] == 'L') --LRDirection;
                            if(direction[i] == 'R') ++LRDirection;
                        } else {
                            if(direction[i] == 'L') ++LRDirection;
                            if(direction[i] == 'R') --LRDirection;
                        }
                    }
                    
                    /* Calculate direction */
                    CVector2 cDir =
                        DIRECTION_VECTOR_FACTOR *
                        (CVector2(FBDirection, 0.0f) +
                        CVector2(0.0f, LRDirection));

                    /* Set direction */
                    cController.SetControlVector(cDir);

                    break;
                }
            }

            if(m_bLogging && timestep > 0) {

                /* Log info if move command has changed */
                if(m_pcLastClientMoveCommands.find(username) == m_pcLastClientMoveCommands.end() ||
                   m_pcLastClientMoveCommands[username] != direction) {

                    m_cOutput.open(m_strCommandFilePath.c_str(), std::ios_base::app);
                    m_cOutput << "\n" << (int)timestep << "," << username << "," << target << ",move," <<  direction;
                    m_cOutput.close();

                    m_pcLastClientMoveCommands[username] = direction;
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
                        std::cout << "[INFO] (" << cController.GetId() << ") sending START task signal" << std::endl;
                        cController.SetSignal(true);
                    } else if(signal == "stop") {
                        std::cout << "[INFO] (" << cController.GetId() << ") sending STOP task signal" << std::endl;
                        cController.SetSignal(false);
                    }

                    break;
                }
            }
        }
        else if(c_data["command"] == "request") {

            int num_robot = c_data["number"];

            std::cout << "Request received: " << num_robot << std::endl;

            /* Get robot controller */
            CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
            for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                it != m_cEPuckLeaders.end();
                ++it) {

                /* Get handle to e-puck_leader entity and controller */
                CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                if(cController.GetId() == target) {
                    
                    /* Tell the e-puck to request robots from the other team */
                    cController.SetRobotsToRequest(num_robot);
                    break;
                }
            }

            if(m_bLogging && timestep > 0) {
                m_cOutput.open(m_strCommandFilePath.c_str(), std::ios_base::app);
                m_cOutput << "\n" << (int)timestep << "," << username << "," << target << ",request," <<  num_robot;
                m_cOutput.close();
            }
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

            if(m_bLogging && timestep > 0) {
                m_cOutput.open(m_strCommandFilePath.c_str(), std::ios_base::app);
                m_cOutput << "\n" << (int)timestep << "," << username << "," << target << ",send," <<  num_robot;
                m_cOutput.close();
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
                    // if(username != "") {
                    //     /* Update username */
                    //     CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
                    //     for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
                    //         it != m_cEPuckLeaders.end();
                    //         ++it) {

                    //         /* Get handle to e-puck_leader entity and controller */
                    //         CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
                    //         CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

                    //         if(cController.GetId() == target) {
                    //             cController.SetUsername(username);
                    //             std::cout << "[LOG] (" << target << ") connected to " << username << " (" << client << ")" << std::endl;
                    //             break;
                    //         }
                    //     }
                    //     m_pcClientRobotConnections[target].username = username;
                    // }
                    // std::cout << "[LOG] (" << target << ") is already controlled by " << clientInControl.username << std::endl;
                    continue;
                } 
                else if(clientInControl.id != "") { 
                    std::cout << "[INFO]: (" << target << ") is already being controlled by " 
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

                        if(cController.GetId() == key && key != target) {
                            cController.Deselect();
                            cController.SetUsername("");
                            cController.SetSignal(false);
                            value = ClientData();
                            std::cout << "[LOG] (" << key << ") released by " << username << "..." << std::endl;
                            break;
                        }
                    }                    
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

                    // std::cout << "[LOG] (" << target << ") connected to " << username << " (" << client << ")" << std::endl;
                    std::cout << "[LOG] (" << target << ") connected to " << username << "..." << std::endl;
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

    /* Send current points obtained */
    outJson["points"] = (int)m_pcExperimentLoopFunctions->GetCurrentPoints();

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
    outJson["num_task_demand"] = cController.GetTaskDemand();
    outJson["num_init_task_demand"] = cController.GetInitTaskDemand();

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
    // std::cout << "[LOG] Adding client " << str_id << std::endl;

    /* Create entry for connected client */
    m_pcClientPointerToId[str_id] = ClientData();

    // std::cout << "[LOG] " << m_pcClientPointerToId.size() << std::endl;

}

/****************************************/
/****************************************/

void CManualControlWebvizUserFunctions::ClientDisconnected(std::string str_id) {
    // std::cout << "[LOG] Disconnected " << str_id << std::endl;

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