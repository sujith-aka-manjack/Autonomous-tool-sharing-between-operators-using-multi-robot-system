#include "experiment_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/e-puck_leader/simulator/epuckleader_entity.h>
#include <controllers/leader/leader.h>
#include <controllers/follower/follower.h>
#include <argos3/plugins/simulator/entities/circle_task_entity.h>
#include <argos3/plugins/simulator/entities/rectangle_task_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

#include <utility/robot_message.h>

#include <fstream>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/delimited_message_util.h>
#include <protos/generated/time_step.pb.h>

/****************************************/
/****************************************/

static const Real        EP_RADIUS        = 0.035f;
static const Real        EP_AREA          = ARGOS_PI * Square(0.035f);
static const Real        EP_RAB_RANGE     = 0.8f;
static const Real        EP_RAB_DATA_SIZE = MESSAGE_BYTE_SIZE;
static const std::string HL_CONTROLLER    = "el";
static const std::string EP_CONTROLLER    = "ef";
static const UInt32      MAX_PLACE_TRIALS = 20;
static const UInt32      MAX_ROBOT_TRIALS = 20;

/****************************************/
/****************************************/

CExperimentLoopFunctions::CExperimentLoopFunctions() :
    m_pcFloor(NULL),
    m_pcRNG(NULL) {
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::Init(TConfigurationNode& t_node) {
    
    std::cout << "Init experiment loop function" << std::endl;

    config = t_node;

    try {
        /*
        * Parse the configuration file
        */
        TConfigurationNode& tChainFormation = GetNode(t_node, "output");
        /* Get a pointer to the floor entity */
        // m_pcFloor = &GetSpace().GetFloorEntity();
        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");
        /* Get the output file name from XML */
        GetNodeAttributeOrDefault(tChainFormation, "logging", m_bLogging, false);
        GetNodeAttributeOrDefault(tChainFormation, "out_path", m_strOutput, std::string("results/experiment.yaml"));
        /* Set the frame grabbing settings */
        GetNodeAttributeOrDefault(tChainFormation, "frame_grabbing", m_bFrameGrabbing, false);
        GetNodeAttributeOrDefault(tChainFormation, "camera_index", m_unCameraIndex, (UInt32)0);

        InitRobots();
        InitTasks();

    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::Reset() {
    std::cout << "RESET called" << std::endl;
    
    /* Delete existing robot and task entities from the simulation */
    for(const auto& id : m_vecEntityID) {
        RemoveEntity(id);
    }

    m_vecEntityID.clear();

    InitRobots();
    InitTasks();
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::Destroy() {
    std::cout << "DESTROY called" << std::endl;
}

/****************************************/
/****************************************/

CColor CExperimentLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
    // for(UInt32 i = 0; i < m_tTasks.size(); ++i) {
    //     if((c_position_on_plane - m_tTasks[i].position).SquareLength() < 0.1 ) {
    //         return CColor::ORANGE;
    //     }
    // }
    // for(UInt32 i = 0; i < m_vecWaypointPos.size(); ++i) {
    //     if((c_position_on_plane - m_vecWaypointPos[i]).SquareLength() < 0.01) {
    //         return CColor::GRAY70;
    //     }
    // }
    return CColor::GRAY90;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::PreStep() {

    // std::cout << "TIME: " << GetSpace().GetSimulationClock() << std::endl;

    UInt32 unFollowers1 = 0;
    UInt32 unFollowers2 = 0;
    UInt32 unConnectors = 0;
    std::unordered_map<std::string,CVector2> leaderPos;
    std::unordered_map<std::string,UInt32> taskWithRobot; // Store the number of e-pucks that have worked on each task in the previous timestep

    /* Add existing task id to the map */
    CSpace::TMapPerType* m_cCTasks;
    bool taskExists;
    try {
        // m_cCTasks = &GetSpace().GetEntitiesByType("circle_task");
        m_cCTasks = &GetSpace().GetEntitiesByType("rectangle_task");
        taskExists = true;
    } catch(CARGoSException& ex) {
        std::cout << "No circle task found in argos file (PreStep)" << std::endl;
        taskExists = false;
    }

    if(taskExists) {
        for(CSpace::TMapPerType::iterator itTask = m_cCTasks->begin();
            itTask != m_cCTasks->end();
            ++itTask) {
            
            /* Initialize each task with zero e-pucks working on it */
            // CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
            CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
            taskWithRobot[cCTask.GetId()] = 0;
        }
    }

    /* Loop leaders */
    CSpace::TMapPerType& m_cEPuckLeaders = GetSpace().GetEntitiesByType("e-puck_leader");
    for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
        it != m_cEPuckLeaders.end();
        ++it) {

        /* Get handle to e-puck_leader entity and controller */
        CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
        CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

        /* Get the position of the leader on the ground as a CVector2 */
        CVector2 cPos = CVector2(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                 cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        leaderPos[cEPuckLeader.GetId()] = cPos;

        bool leaderAtTask = false;

        /* Give the leader its next task info if it is within the task range */        
        if(taskExists) {

            for(CSpace::TMapPerType::iterator itTask = m_cCTasks->begin();
                itTask != m_cCTasks->end();
                ++itTask) {

                /* Task location */
                // CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

                CVector2 cTaskPos = cCTask.GetPosition();

                // /* If there is a task with the given task position AND leader is within the task range, return task demand */
                // if((cPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2)) {
                //     cController.SetTaskDemand(cCTask.GetDemand());
                //     break;
                // }

                /* If there is a task with the given task position AND leader is within the task range, return task demand */
                if(cCTask.InArea(cPos)) {
                    cController.SetTaskDemand(cCTask.GetDemand());
                    cController.SetInitTaskDemand(cCTask.GetInitDemand());
                    cController.SetMinimumCount(cCTask.GetMinRobotNum());
                    leaderAtTask = true;
                    break;
                }
            }
        }

        /* Reset task info if the leader is not at any task */
        if( !leaderAtTask ) {
            cController.SetTaskDemand(0);
            cController.SetInitTaskDemand(0);
            cController.SetMinimumCount(0);
        }
    }

    /* Loop followers */
    CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
        itEpuck != m_cEPucks.end();
        ++itEpuck) {

        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
        try {
            CFollower& cController = dynamic_cast<CFollower&>(cEPuck.GetControllableEntity().GetController());
            UInt8 unTeamId = cController.GetTeamID();

            /* Count how many e-pucks are in each state */
            if( cController.GetRobotState() == RobotState::FOLLOWER ) {
                // Count flock state
                if( unTeamId == 1 ) ++unFollowers1;
                else ++unFollowers2;

                /* 
                * Check whether the e-puck is working on a task
                */            

                /* Current location */
                CVector2 cPos = CVector2(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                         cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

                /* Leaeder's location */
                std::ostringstream cLeaderId;
                cLeaderId.str("");
                cLeaderId << "L" << unTeamId;
                CVector2 cLeaderPos = leaderPos[cLeaderId.str()];

                if(taskExists) {

                    for(CSpace::TMapPerType::iterator itTask = m_cCTasks->begin();
                        itTask != m_cCTasks->end();
                        ++itTask) {

                        /* Task location */
                        // CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                        CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

                        CVector2 cTaskPos = cCTask.GetPosition();

                        /* Check if robot is working on a task */
                        if(cController.IsWorking()) {
                            /* Check e-puck and its leader is within the range of a task */
                            // if((cPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2) &&
                            // (cLeaderPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2)) {
                                
                            //     taskWithRobot[cCTask.GetId()]++; // Increment robot working on this task
                            //     break;
                            // }
                            if(cCTask.InArea(cPos) && cCTask.InArea(cLeaderPos)) {
                                
                                taskWithRobot[cCTask.GetId()]++; // Increment robot working on this task
                                break;
                            }
                        }
                    }
                }
            }
            else 
                ++unConnectors; // Count the number of connectors

        } catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("While casting robot as a follower", ex);
            
        } catch(const std::bad_cast& e) {
            std::cout << e.what() << " in PreStep" << '\n';

        }
    }

    /* Inform each leader the number of followers in its team */
    for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
        it != m_cEPuckLeaders.end();
        ++it) {

        /* Get handle to e-puck_leader entity and controller */
        CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
        CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

        if(cController.GetId() == "L1")
            cController.SetFollowerCount(unFollowers1);
        else if(cController.GetId() == "L2")
            cController.SetFollowerCount(unFollowers2);
    }

    /* Update task demands */
    if(taskExists) {

        for(CSpace::TMapPerType::iterator itTask = m_cCTasks->begin();
            itTask != m_cCTasks->end();
            ++itTask) {

            // CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
            CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

            UInt32 currentDemand = cCTask.GetDemand();

            /* Check if there is enough robots working on the task */
            if(taskWithRobot[cCTask.GetId()] >= cCTask.GetMinRobotNum()) {
                if(currentDemand < taskWithRobot[cCTask.GetId()]) {
                    cCTask.SetDemand(0);
                } else {
                    // cCTask.SetDemand(currentDemand - taskWithRobot[cCTask.GetId()]);
                    cCTask.SetDemand(currentDemand - 1);
                }
            }

            cCTask.SetCurrentRobotNum(taskWithRobot[cCTask.GetId()]);
        }
    }

    /* 
    * Output stuff to file 
    */

    if(m_bLogging) {
        /* Create new node for this timestep */
        TimeStep tData;
        tData.set_time(GetSpace().GetSimulationClock());

        /* Output leader info */
        for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
            it != m_cEPuckLeaders.end();
            ++it) {

            CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
            CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

            Robot* robot = tData.add_robots();
            robot->set_name(cEPuckLeader.GetId());
            robot->set_teamid(cController.GetTeamID());
            robot->set_state(Robot_State_LEADER);
            robot->mutable_position()->set_x(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Position.GetX());
            robot->mutable_position()->set_y(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            robot->set_totalsent((int)cController.GetTotalSent());
            robot->set_totalreceived((int)cController.GetTotalReceived());
            robot->set_action(cController.GetLastAction());
        }

        /* Output follower info */
        for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
            itEpuck != m_cEPucks.end();
            ++itEpuck) {

            CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
            CFollower& cController = dynamic_cast<CFollower&>(cEPuck.GetControllableEntity().GetController());

            Robot* robot = tData.add_robots();
            robot->set_name(cEPuck.GetId());
            robot->set_teamid(cController.GetTeamID());
            switch(cController.GetRobotState()) {
                case RobotState::FOLLOWER:
                    robot->set_state(Robot_State_FOLLOWER);
                    break;
                case RobotState::CONNECTOR:
                    robot->set_state(Robot_State_CONNECTOR);
                    break;
                case RobotState::TRAVELER:
                    robot->set_state(Robot_State_TRAVELER);
                    break;
                default:
                    std::cerr << "Tried to log unknown state " << int(cController.GetRobotState()) << std::endl;
                    break;
            }
            robot->mutable_position()->set_x(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX());
            robot->mutable_position()->set_y(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        }

        if(taskExists) {

            /* Output task info */
            for(CSpace::TMapPerType::iterator itTask = m_cCTasks->begin();
                itTask != m_cCTasks->end();
                ++itTask) {

                CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

                Task* task = tData.add_tasks();
                task->set_name(cCTask.GetId());
                task->set_demand(cCTask.GetDemand());
                task->set_requiredrobots(cCTask.GetMinRobotNum());
                task->set_currentrobots(cCTask.GetCurrentRobotNum());
            }
        }

        /* Write to file */
        m_cOutput.open(m_strOutput.c_str(), std::ios::app | std::ios::binary);
        google::protobuf::util::SerializeDelimitedToOstream(tData, &m_cOutput);
        m_cOutput.close();
    }

    /* Grab frame */
    if(m_bFrameGrabbing) {
        CQTOpenGLRender& render = dynamic_cast<CQTOpenGLRender&>(GetSimulator().GetVisualization());
        CQTOpenGLWidget& widget = render.GetMainWindow().GetOpenGLWidget();
        widget.SetCamera(m_unCameraIndex);
        widget.SetGrabFrame(m_bFrameGrabbing);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::PostStep() {
    Real total_demand = 0;

    /* Add existing task id to the map */
    CSpace::TMapPerType* m_cCTasks;
    bool taskExists;
    try {
        // m_cCTasks = &GetSpace().GetEntitiesByType("circle_task");
        m_cCTasks = &GetSpace().GetEntitiesByType("rectangle_task");
        taskExists = true;
    } catch(CARGoSException& ex) {
        std::cout << "No circle task found in argos file (PreStep)" << std::endl;
        taskExists = false;
    }

    if(taskExists) {
        for(CSpace::TMapPerType::iterator itTask = m_cCTasks->begin();
            itTask != m_cCTasks->end();
            ++itTask) {
            
            /* Initialize each task with zero e-pucks working on it */
            // CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
            CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
            total_demand += (int)cCTask.GetDemand();
        }
    }

    if(total_demand == 0) {
        CSimulator::GetInstance().Terminate();
        std::cout << "[LOG] TERMINATING SIMULATION ..." << std::endl;
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::InitRobots() {
    /*
    * Distribute leaders and robots
    */

    std::cout << "[LOG] Adding robots..." << std::endl;

    /* ID counts */
    UInt32 unNextLeaderId = 1;
    UInt32 unNextRobotId = 1;
    /* Get the teams node */
    TConfigurationNode& et_tree = GetNode(config, "teams");
    /* Go through the nodes (teams) */
    TConfigurationNodeIterator itDistr;

    for(itDistr = itDistr.begin(&et_tree);
        itDistr != itDistr.end();
        ++itDistr) {
        
        /* Get current node (team/custom_team) */
        TConfigurationNode& tDistr = *itDistr;

        /* Number of leaders to place */
        UInt32 unLeaders = 0;

        /* Number of robots to place */
        UInt32 unRobots;

        if(itDistr->Value() == "team") {
            /* Distribution center */
            CVector2 cCenter;
            GetNodeAttribute(tDistr, "center", cCenter);
            GetNodeAttribute(tDistr, "leader_num", unLeaders);
            GetNodeAttribute(tDistr, "robot_num", unRobots);
            /* Density of the robots */
            Real fDensity;
            GetNodeAttribute(tDistr, "density", fDensity);
            /* Place robots */
            PlaceCluster(cCenter, unLeaders, unRobots, fDensity, unNextLeaderId, unNextRobotId);

            /* Get the waypoints node */
            std::queue<CVector2> waypoints; // Queue to provide to the robot
            /* Go through the nodes (waypoints) */
            TConfigurationNodeIterator itWaypt;
            for(itWaypt = itWaypt.begin(&tDistr);
                itWaypt != itWaypt.end();
                ++itWaypt) {

                /* Get current node (waypoint) */
                TConfigurationNode& tWaypt = *itWaypt;
                /* Coordinate of waypoint */
                CVector2 coord;
                GetNodeAttribute(tWaypt, "coord", coord);
                m_vecWaypointPos.push_back(coord);
                waypoints.push(coord);
            }

            /* Get the newly created leader */
            std::ostringstream cEPId;
            cEPId.str("");
            cEPId << "L" << unNextLeaderId;
            CEPuckLeaderEntity& cEPuckLeader = dynamic_cast<CEPuckLeaderEntity&>(GetSpace().GetEntity(cEPId.str()));
            CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());
            /* Set list of waypoints to leader */
            cController.SetWaypoints(waypoints);

            /* Update robot count */
            unNextLeaderId += unLeaders;
            unNextRobotId += unRobots;
        }
        else if(itDistr->Value() == "custom_team") {
            
            /* Get the robots node */
            TConfigurationNode& er_tree = GetNode(tDistr, "robots");
            /* Go through the nodes (robots) */
            TConfigurationNodeIterator itRobot;

            for(itRobot = itRobot.begin(&er_tree);
                itRobot != itRobot.end();
                ++itRobot) {

                /* Get current node (robot) */
                TConfigurationNode& tRobot = *itRobot;
                /* Distribution center */
                CVector2 cCenter;
                GetNodeAttribute(tRobot, "position", cCenter);
                std::string str_type;
                GetNodeAttribute(tRobot, "type", str_type);

                PlaceCustomPosition(cCenter, str_type, unNextLeaderId, unNextRobotId);

                if(str_type == "leader")
                    unLeaders++;
                else if(str_type == "follower")
                    unNextRobotId++;
            }

            /* Get the waypoints node */
            std::queue<CVector2> waypoints; // Queue to provide to the robot
            TConfigurationNode& ew_tree = GetNode(tDistr, "waypoints");
            /* Go through the nodes (waypoints) */
            TConfigurationNodeIterator itWaypt;
            for(itWaypt = itWaypt.begin(&ew_tree);
                itWaypt != itWaypt.end();
                ++itWaypt) {

                /* Get current node (waypoint) */
                TConfigurationNode& tWaypt = *itWaypt;
                /* Coordinate of waypoint */
                CVector2 coord;
                GetNodeAttribute(tWaypt, "coord", coord);
                m_vecWaypointPos.push_back(coord);
                waypoints.push(coord);
            }

            /* Get the newly created leader */
            std::ostringstream cEPId;
            cEPId.str("");
            cEPId << "L" << unNextLeaderId;
            CEPuckLeaderEntity& cEPuckLeader = dynamic_cast<CEPuckLeaderEntity&>(GetSpace().GetEntity(cEPId.str()));
            CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());
            /* Set list of waypoints to leader */
            cController.SetWaypoints(waypoints);

            /* Update robot count */
            unNextLeaderId += unLeaders;
        } 
    }

    std::cout << "[LOG] Added robots" << std::endl;

}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::InitTasks() {
    /*
    * Initialize tasks
    */

    std::cout << "[LOG] Adding tasks..." << std::endl;

    /* ID counts */
    UInt32 unNextTaskId = 1;
    /* Get the teams node */
    TConfigurationNode& ts_tree = GetNode(config, "tasks");
    /* Go through the nodes (tasks) */
    TConfigurationNodeIterator itDistr;
    for(itDistr = itDistr.begin(&ts_tree);
        itDistr != itDistr.end();
        ++itDistr) {

        // /* Get current node (task) */
        // TConfigurationNode& tDistr = *itDistr;
        // /* Task center */
        // CVector2 cCenter;
        // GetNodeAttribute(tDistr, "position", cCenter);
        // /* Task radius */
        // Real fRadius;
        // GetNodeAttribute(tDistr, "radius", fRadius);
        // /* Task demand */
        // UInt32 unDemand;
        // GetNodeAttribute(tDistr, "task_demand", unDemand);
        // /* Minimum robot constraint */
        // UInt32 unMinRobotNum;
        // GetNodeAttribute(tDistr, "minimum_robot_num", unMinRobotNum);
        // /* Maximum robot constraint */
        // UInt32 unMaxRobotNum;
        // GetNodeAttribute(tDistr, "maximum_robot_num", unMaxRobotNum);
        
        // /* Place Tasks */
        // PlaceTask(cCenter, fRadius, unDemand, unMinRobotNum, unMaxRobotNum, unNextTaskId);

        // /* Update task count */
        // unNextTaskId++;

        /* Get current node (task) */
        TConfigurationNode& tDistr = *itDistr;
        /* Task center */
        CVector2 cCenter;
        GetNodeAttribute(tDistr, "position", cCenter);
        /* Task width */
        Real fWidth;
        GetNodeAttribute(tDistr, "width", fWidth);
        /* Task height */
        Real fHeight;
        GetNodeAttribute(tDistr, "height", fHeight);
        /* Task demand */
        UInt32 unDemand;
        GetNodeAttribute(tDistr, "task_demand", unDemand);
        /* Minimum robot constraint */
        UInt32 unMinRobotNum;
        GetNodeAttribute(tDistr, "minimum_robot_num", unMinRobotNum);
        /* Maximum robot constraint */
        UInt32 unMaxRobotNum;
        GetNodeAttribute(tDistr, "maximum_robot_num", unMaxRobotNum);
        
        /* Place Tasks */
        PlaceRectangleTask(cCenter, fWidth, fHeight, unDemand, unMinRobotNum, unMaxRobotNum, unNextTaskId);

        /* Update task count */
        unNextTaskId++;
    }

    std::cout << "[LOG] Added tasks" << std::endl;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::PlaceCluster(const CVector2& c_center,
                                            UInt32 un_leaders,
                                            UInt32 un_robots,
                                            Real f_density,
                                            UInt32 un_leader_id_start,
                                            UInt32 un_robot_id_start) {

    try {
        /* Calculate side of the region in which the robots are scattered */
        Real fHalfSide = Sqrt((EP_AREA * un_robots) / f_density) / 2.0f;
        CRange<Real> cAreaRange(-fHalfSide, fHalfSide);
        /* Place robots */
        UInt32 unTrials;
        CEPuckLeaderEntity* pcEPL;
        CEPuckEntity* pcEP;
        std::ostringstream cEPId;
        CVector3 cEPPos;
        CQuaternion cEPRot;

        /* For each leader */ // CURRENTLY ONLY ONE LEADER PER TEAM IS SUPPORTED
        for(size_t i = 0; i < un_leaders; ++i) {
            /* Make the id */
            cEPId.str("");
            cEPId << "L" << (i + un_leader_id_start);
            /* Create the leader in the origin and add it to ARGoS space */
            pcEPL = new CEPuckLeaderEntity(cEPId.str(),
                                           HL_CONTROLLER,
                                           CVector3(),
                                           CQuaternion(),
                                           EP_RAB_RANGE,
                                           EP_RAB_DATA_SIZE,
                                           "");
            AddEntity(*pcEPL);
            m_vecEntityID.push_back(cEPId.str());

            /* Try to place it in the arena */
            unTrials = 0;
            bool bDone;
            do {
                /* Choose a random position */
                ++unTrials;
                cEPPos.Set(m_pcRNG->Uniform(cAreaRange) + c_center.GetX(),
                           m_pcRNG->Uniform(cAreaRange) + c_center.GetY(),
                           0.0f);
                cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                     CVector3::Z);
                bDone = MoveEntity(pcEPL->GetEmbodiedEntity(), cEPPos, cEPRot);
            } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
            if(!bDone) {
                THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
            }
        }

        /* For each robot */
        for(size_t i = 0; i < un_robots; ++i) {
            /* Make the id */
            cEPId.str("");
            cEPId << "F" << (i + un_robot_id_start);
            /* Create the robot in the origin and add it to ARGoS space */
            pcEP = new CEPuckEntity(cEPId.str(),
                                    EP_CONTROLLER,
                                    CVector3(),
                                    CQuaternion(),
                                    EP_RAB_RANGE,
                                    EP_RAB_DATA_SIZE,
                                    "");
            AddEntity(*pcEP);
            m_vecEntityID.push_back(cEPId.str());

            /* Assign initial team id */
            CFollower& cController = dynamic_cast<CFollower&>(pcEP->GetControllableEntity().GetController());
            cController.SetTeamID(un_leader_id_start);  // Assign teamID of first team leader
            /* Try to place it in the arena */
            unTrials = 0;
            bool bDone;
            do {
                /* Choose a random position */
                ++unTrials;
                cEPPos.Set(m_pcRNG->Uniform(cAreaRange) + c_center.GetX(),
                           m_pcRNG->Uniform(cAreaRange) + c_center.GetY(),
                           0.0f);
                cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                     CVector3::Z);
                bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);
            } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
            if(!bDone) {
                THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
            }
        }
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing robots in a cluster", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::PlaceCustomPosition(const CVector2& c_center,
                                                   std::string str_type,
                                                   UInt32 un_leader_id_start,
                                                   UInt32 un_robot_id_start) {

    try{
        /* Place robot */
        std::ostringstream cEPId;
        CVector3 cEPPos;
        CQuaternion cEPRot;

        if(str_type == "leader") {
            CEPuckLeaderEntity* pcEPL;
            /* Make the id */
            cEPId.str("");
            cEPId << "L" << (un_leader_id_start);
            /* Create the leader in the origin and add it to ARGoS space */
            pcEPL = new CEPuckLeaderEntity(cEPId.str(),
                                           HL_CONTROLLER,
                                           CVector3(),
                                           CQuaternion(),
                                           EP_RAB_RANGE,
                                           EP_RAB_DATA_SIZE,
                                           "");
            AddEntity(*pcEPL);
            m_vecEntityID.push_back(cEPId.str());

            /* Try to place it in the arena */
            bool bDone;
            /* Place on specified position */
            cEPPos.Set(c_center.GetX(),
                        c_center.GetY(),
                        0.0f);
            cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                    CVector3::Z);
            bDone = MoveEntity(pcEPL->GetEmbodiedEntity(), cEPPos, cEPRot);

            if(!bDone) {
                THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
            }
        }
        else if(str_type == "follower") {
            CEPuckEntity* pcEP;
            /* Make the id */
            cEPId.str("");
            cEPId << "F" << (un_robot_id_start);
            /* Create the robot in the origin and add it to ARGoS space */
            pcEP = new CEPuckEntity(cEPId.str(),
                                    EP_CONTROLLER,
                                    CVector3(),
                                    CQuaternion(),
                                    EP_RAB_RANGE,
                                    EP_RAB_DATA_SIZE,
                                    "");
            AddEntity(*pcEP);
            m_vecEntityID.push_back(cEPId.str());

            /* Assign initial team id */
            CFollower& cController = dynamic_cast<CFollower&>(pcEP->GetControllableEntity().GetController());
            cController.SetTeamID(un_leader_id_start);  // Assign teamID of team leader

            /* Try to place it in the arena */
            bool bDone;
            /* Place on specified position */
            cEPPos.Set(c_center.GetX(),
                        c_center.GetY(),
                        0.0f);
            cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                    CVector3::Z);
            bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);
        }
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing robot in a custom position", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::PlaceTask(const CVector2& c_center,
                                         Real f_radius,
                                         UInt32 un_demand,
                                         UInt32 un_min_robot_num,
                                         UInt32 un_max_robot_num,
                                         UInt32 un_task_id_start) {

    try {
        CCircleTaskEntity* pcCTS;
        std::ostringstream cTSId;

        /* Make the id */
        cTSId.str("");
        cTSId << "task_" << un_task_id_start;
        /* Create the task and add it to ARGoS space */
        pcCTS = new CCircleTaskEntity(cTSId.str(),
                                      c_center,
                                      f_radius,
                                      un_demand,
                                      un_min_robot_num,
                                      un_max_robot_num);
        AddEntity(*pcCTS);
        m_vecEntityID.push_back(cTSId.str());

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a task", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::PlaceRectangleTask(const CVector2& c_center,
                                         Real f_width,
                                         Real f_height,
                                         UInt32 un_demand,
                                         UInt32 un_min_robot_num,
                                         UInt32 un_max_robot_num,
                                         UInt32 un_task_id_start) {

    try {
        CRectangleTaskEntity* pcRTS;
        std::ostringstream cTSId;

        /* Make the id */
        cTSId.str("");
        cTSId << "task_" << un_task_id_start;
        /* Create the task and add it to ARGoS space */
        pcRTS = new CRectangleTaskEntity(cTSId.str(),
                                      c_center,
                                      f_width,
                                      f_height,
                                      un_demand,
                                      un_min_robot_num,
                                      un_max_robot_num);
        AddEntity(*pcRTS);
        m_vecEntityID.push_back(cTSId.str());

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a task", ex);
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CExperimentLoopFunctions, "experiment_loop_functions")
