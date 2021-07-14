#include "experiment_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <e-puck_leader/simulator/epuckleader_entity.h>
#include <controllers/leader/leader.h>
#include <controllers/follower/follower.h>
#include <circle_task/circle_task_entity.h>

/****************************************/
/****************************************/

static const Real        EP_RADIUS        = 0.035f;
static const Real        EP_AREA          = ARGOS_PI * Square(0.035f);
static const Real        EP_RAB_RANGE     = 0.8f;
static const Real        EP_RAB_DATA_SIZE = 81;
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

    try {
        /*
        * Parse the configuration file
        */
        TConfigurationNode& tForaging = GetNode(t_node, "experiment");
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");
        /* Get the output file name from XML */
        GetNodeAttribute(tForaging, "output", m_strOutput);
        /* Open the file, erasing its contents */
        m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
        m_cOutput << "# clock,"
                     "leader1posX,"
                     "leader1posY,"
                     "leader1follower,"
                     "leader2posX,"
                     "leader2posY,"
                     "leader2follower,"
                     "connector" << std::endl;

        /*
        * Distribute leaders and robots
        */

        std::cout << "[LOG] Adding robots..." << std::endl;

        /* ID counts */
        UInt32 unNextLeaderId = 1;
        UInt32 unNextRobotId = 1;
        /* Get the teams node */
        TConfigurationNode& et_tree = GetNode(t_node, "teams");
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
                    m_cWaypointPos.push_back(coord);
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

                /* Update robot count */
                unNextLeaderId += unLeaders;
            } 
        }

        std::cout << "[LOG] Added robots" << std::endl;

        /*
        * Initialize tasks
        */

        std::cout << "[LOG] Adding tasks..." << std::endl;

        /* ID counts */
        UInt32 unNextTaskId = 1;
        /* Get the teams node */
        TConfigurationNode& ts_tree = GetNode(t_node, "tasks");
        /* Go through the nodes (tasks) */
        for(itDistr = itDistr.begin(&ts_tree);
            itDistr != itDistr.end();
            ++itDistr) {

            /* Get current node (task) */
            TConfigurationNode& tDistr = *itDistr;
            /* Task center */
            CVector2 cCenter;
            GetNodeAttribute(tDistr, "position", cCenter);
            /* Task radius */
            Real fRadius;
            GetNodeAttribute(tDistr, "radius", fRadius);
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
            PlaceTask(cCenter, fRadius, unDemand, unMinRobotNum, unMaxRobotNum, unNextTaskId);

            /* Update task count */
            unNextTaskId++;
        }

        std::cout << "[LOG] Added tasks" << std::endl;
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::Reset() {
    /* Close the file */
    m_cOutput.close();
    /* Open the file, erasing its contents */
    m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
    m_cOutput << "# clock,"
                 "leader1posX,"
                 "leader1posY,"
                 "leader1follower,"
                 "leader2posX,"
                 "leader2posY,"
                 "leader2follower,"
                 "connector" << std::endl;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::Destroy() {
    /* Close the file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CExperimentLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
    // for(UInt32 i = 0; i < m_tTasks.size(); ++i) {
    //     if((c_position_on_plane - m_tTasks[i].position).SquareLength() < 0.1 ) {
    //         return CColor::ORANGE;
    //     }
    // }
    // for(UInt32 i = 0; i < m_cWaypointPos.size(); ++i) {
    //     if((c_position_on_plane - m_cWaypointPos[i]).SquareLength() < 0.01) {
    //         return CColor::GRAY70;
    //     }
    // }
    return CColor::GRAY90;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::PreStep() {

    UInt32 unFollowers1 = 0;
    UInt32 unFollowers2 = 0;
    UInt32 unConnectors = 0;
    std::unordered_map<std::string,CVector2> leaderPos;
    std::unordered_map<std::string,UInt32> taskWithRobot; // Store the number of e-pucks that have worked on each task in the previous timestep

    /* Add existing task id to the map */
    CSpace::TMapPerType& m_cCTasks = GetSpace().GetEntitiesByType("circle_task");
    for(CSpace::TMapPerType::iterator itTask = m_cCTasks.begin();
        itTask != m_cCTasks.end();
        ++itTask) {
        
        /* Initialize each task with zero e-pucks working on it */
        CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
        taskWithRobot[cCTask.GetId()] = 0;
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

        /* Give the leader its next task info if it is within the task range */
        CVector2 nextTaskPos = cController.GetNextWaypoint();
        for(CSpace::TMapPerType::iterator itTask = m_cCTasks.begin();
            itTask != m_cCTasks.end();
            ++itTask) {

            /* Task location */
            CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
            CVector2 cTaskPos = cCTask.GetPosition();

            /* If there is a task with the given task position AND leader is within the task range, return task demand */
            if(nextTaskPos == cTaskPos && (cPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2)) {
                cController.SetTaskDemand(cCTask.GetDemand());
                break;
            }
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
            if( cController.currentState == CFollower::RobotState::FOLLOWER ) {
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

                for(CSpace::TMapPerType::iterator itTask = m_cCTasks.begin();
                    itTask != m_cCTasks.end();
                    ++itTask) {

                    /* Task location */
                    CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                    CVector2 cTaskPos = cCTask.GetPosition();

                    /* Check if robot is working on a task */
                    if(cController.IsWorking()) {
                        /* Check e-puck and its leader is within the range of a task */
                        if((cPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2) &&
                        (cLeaderPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2)) {
                            
                            taskWithRobot[cCTask.GetId()]++; // Increment robot working on this task
                            break;
                        }
                    }
                }
            }
            else 
                ++unConnectors; // Count the number of connectors

        } catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("While casting robot as a follower", ex);
            
        } catch(const std::bad_cast& e) {
            std::cout << e.what() << '\n';

        }
    }

    /* Update task demands */
    for(CSpace::TMapPerType::iterator itTask = m_cCTasks.begin();
        itTask != m_cCTasks.end();
        ++itTask) {

        CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
        UInt32 currentDemand = cCTask.GetDemand();

        /* Check if there is enough robots working on the task */
        if(taskWithRobot[cCTask.GetId()] >= cCTask.GetMinRobotNum()) {
            if(currentDemand < taskWithRobot[cCTask.GetId()]) {
                cCTask.SetDemand(0);
            } else {
                cCTask.SetDemand(currentDemand - taskWithRobot[cCTask.GetId()]);
            }
        }

        // TODO: Record task demand
    }

    /* Output stuff to file */
    m_cOutput << GetSpace().GetSimulationClock() << ","
              << leaderPos["L1"].GetX() << ","
              << leaderPos["L1"].GetY() << ","
              << unFollowers1 << ","
              << leaderPos["L2"].GetX() << ","
              << leaderPos["L2"].GetY() << ","
              << unFollowers2 << ","
              << unConnectors << std::endl;
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

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a task", ex);
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CExperimentLoopFunctions, "experiment_loop_functions")
