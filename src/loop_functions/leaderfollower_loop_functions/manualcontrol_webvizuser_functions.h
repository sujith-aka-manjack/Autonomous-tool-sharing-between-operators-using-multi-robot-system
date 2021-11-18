#ifndef MANUALCONTROL_WEBVIZUSER_FUNCTIONS_H
#define MANUALCONTROL_WEBVIZUSER_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck_leader/simulator/epuckleader_entity.h>
#include <argos3/plugins/simulator/visualizations/webviz/webviz_user_functions.h>
#include <loop_functions/leaderfollower_loop_functions/experiment_loop_functions.h>
#include <controllers/leader/leader.h>

#include <iostream>

using namespace argos;

class CManualControlWebvizUserFunctions : public CWebvizUserFunctions {
    public:
        CManualControlWebvizUserFunctions();

        virtual ~CManualControlWebvizUserFunctions();

        // virtual const nlohmann::json sendUserData();

        virtual void HandleCommandFromClient(const std::string& str_ip, nlohmann::json c_json_command);

        // virtual void ClientConnected(std::string str_id);

        // virtual void ClientDisconnected(std::string str_id);

    private:

        CExperimentLoopFunctions *m_pcExperimentLoopFunctions;

        /* 
         * Map of connections between robots and clients 
         * Key is the robot ID. Value is client ID.
         */
        std::map<std::string, std::string> m_pcClientRobotConnections;
};

#endif