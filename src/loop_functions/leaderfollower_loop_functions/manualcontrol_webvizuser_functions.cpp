#include "manualcontrol_webvizuser_functions.h"

/****************************************/
/****************************************/

CManualControlWebvizUserFunctions::CManualControlWebvizUserFunctions() {
//   m_pcForagingLoopFunctions = static_cast<CForagingLoopFunctions *>(
//     &CSimulator::GetInstance().GetLoopFunctions());
}

/****************************************/
/****************************************/

CManualControlWebvizUserFunctions::~CManualControlWebvizUserFunctions() {}

/****************************************/
/****************************************/

// const nlohmann::json CManualControlWebvizUserFunctions::sendExtraData() {
//   return m_pcForagingLoopFunctions->GetStatus();
// }

void CManualControlWebvizUserFunctions::HandleCommandFromClient(const std::string& str_ip, 
                                                                nlohmann::json c_json_command) {


    std::string robot_command = c_json_command["command"];
    std::string direction = c_json_command["direction"];

    std::cout << "From client: " << direction << std::endl;

}

/****************************************/
/****************************************/

REGISTER_WEBVIZ_USER_FUNCTIONS(
  CManualControlWebvizUserFunctions, "manualcontrol_webvizuser_functions")