#include "manualcontrol_webvizuser_functions.h"

/****************************************/
/****************************************/

static const Real DIRECTION_VECTOR_FACTOR = 10.;

/****************************************/
/****************************************/

CManualControlWebvizUserFunctions::CManualControlWebvizUserFunctions() {
    m_pcExperimentLoopFunctions = static_cast<CExperimentLoopFunctions *>(
        &CSimulator::GetInstance().GetLoopFunctions());
}

/****************************************/
/****************************************/

CManualControlWebvizUserFunctions::~CManualControlWebvizUserFunctions() {}

/****************************************/
/****************************************/

const nlohmann::json CManualControlWebvizUserFunctions::sendUserData() {
    
}

/****************************************/
/****************************************/

void CManualControlWebvizUserFunctions::HandleCommandFromClient(const std::string& str_ip, 
                                                                nlohmann::json c_json_command) {

    if(c_json_command.empty())
        return;

    std::string command = c_json_command["command"];

    if(command == "move") {

        // 1) Determine which robot the command is for

        std::string target = c_json_command["robot"];
        std::string direction = c_json_command["direction"];

        // std::cout << "From client: " << direction << std::endl;

        // 2) Get robot controller

        CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
        for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
            it != m_cEPuckLeaders.end();
            ++it) {

            /* Get handle to e-puck_leader entity and controller */
            CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
            CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

            if(cController.GetId() == target) {
                
                // 3) Determine the direction factor

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

                // 4) Set direction
                /* Tell that e-puck that it is selected */
                cController.Select();
                cController.SetControlVector(cDir);

                return;
            }
        }
    }
    else if(command == "select_leader") {

        std::string target = c_json_command["robot"];

        if(target == "Select leader") {
            // Deselect
            // delete from others
            // add id to default
            
            // m_pcClientRobotConnections;
            return;
        }

        std::cout << "Selected " << target << std::endl;

        CSpace::TMapPerType& m_cEPuckLeaders = m_pcExperimentLoopFunctions->GetSpace().GetEntitiesByType("e-puck_leader");
        for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
            it != m_cEPuckLeaders.end();
            ++it) {

            /* Get handle to e-puck_leader entity and controller */
            CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
            CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

            if(cController.GetId() == target) {
                cController.Select();
                // m_pcClientRobotConnections;
                return;
            }
        }
    }
}

/****************************************/
/****************************************/

void CManualControlWebvizUserFunctions::ClientConnected(std::string str_id) {
    std::cout << "Adding client " << str_id << std::endl;

    /* Remove key from map */
    m_pcClientRobotConnections["default"].push_back(str_id);

    for(auto x : m_pcClientRobotConnections["default"]) {
        std::cout << x << std::endl;
    }
}

/****************************************/
/****************************************/

void CManualControlWebvizUserFunctions::ClientDisconnected(std::string str_id) {
    std::cout << "Deleting client " << str_id << std::endl;

    /* Remove key from map */
    auto itr = std::find(m_pcClientRobotConnections["default"].begin(), m_pcClientRobotConnections["default"].end(), str_id);
    if (itr != m_pcClientRobotConnections["default"].end()) m_pcClientRobotConnections["default"].erase(itr);

    for(auto x : m_pcClientRobotConnections["default"]) {
        std::cout << x << std::endl;
    }
}

/****************************************/
/****************************************/

REGISTER_WEBVIZ_USER_FUNCTIONS(
  CManualControlWebvizUserFunctions, "manualcontrol_webvizuser_functions")