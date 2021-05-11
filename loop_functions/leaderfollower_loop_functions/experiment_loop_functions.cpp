#include "experiment_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/leader/leader.h>
#include <controllers/follower/follower.h>

#include <unordered_map>

/****************************************/
/****************************************/

static const Real        EP_RADIUS        = 0.035f;
static const Real        EP_AREA          = ARGOS_PI * Square(0.035f);
static const Real        EP_RAB_RANGE     = 0.8f;
static const Real        EP_RAB_DATA_SIZE = 16;
static const std::string HL_CONTROLLER    = "el";
static const std::string EP_CONTROLLER    = "ef";
static const UInt32      MAX_PLACE_TRIALS = 20;
static const UInt32      MAX_ROBOT_TRIALS = 20;

/****************************************/
/****************************************/

CExperimentLoopFunctions::CExperimentLoopFunctions() :
   /* m_cForagingArenaSideX(-0.9f, 1.7f),
   m_cForagingArenaSideY(-1.7f, 1.7f),
   m_pcFloor(NULL), */
   m_pcRNG(NULL)
   /* m_unCollectedFood(0),
   m_nEnergy(0),
   m_unEnergyPerFoodItem(1),
   m_unEnergyPerWalkingRobot(1) */ {
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
//       /* Get a pointer to the floor entity */
//       m_pcFloor = &GetSpace().GetFloorEntity();
//       /* Get the number of food items we want to be scattered from XML */
//       UInt32 unFoodItems;
//       GetNodeAttribute(tForaging, "items", unFoodItems);
//       /* Get the number of food items we want to be scattered from XML */
//       GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
//       m_fFoodSquareRadius *= m_fFoodSquareRadius;
        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");
//       /* Distribute uniformly the items in the environment */
//       for(UInt32 i = 0; i < unFoodItems; ++i) {
//          m_cFoodPos.push_back(
//             CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
//                      m_pcRNG->Uniform(m_cForagingArenaSideY)));
//       }
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
                     "chain" << std::endl;

        /*
        * Distribute leaders and robots
        */

        /* ID counts */
        UInt32 unPlacedLeaders = 1;
        UInt32 unPlacedRobots = 1;
        /* Get the teams node */
        TConfigurationNode& et_tree = GetNode(t_node, "teams");
        /* Go through the nodes (teams) */
        TConfigurationNodeIterator itDistr;
        for(itDistr = itDistr.begin(&et_tree);
            itDistr != itDistr.end();
            ++itDistr) {
            
            /* Get current node (team) */
            TConfigurationNode& tDistr = *itDistr;
            /* Distribution center */
            CVector2 cCenter;
            GetNodeAttribute(tDistr, "center", cCenter);
            /* Number of leaders to place */
            UInt32 unLeaders;
            GetNodeAttribute(tDistr, "leader_num", unLeaders);
            /* Number of robots to place */
            UInt32 unRobots;
            GetNodeAttribute(tDistr, "robot_num", unRobots);
            /* Density of the robots */
            Real fDensity;
            GetNodeAttribute(tDistr, "density", fDensity);
            /* Place robots */
            PlaceCluster(cCenter, unLeaders, unRobots, fDensity, unPlacedLeaders, unPlacedRobots);

            /* Get the waypoints node */
            std::queue<CVector2> waypoints;
            TConfigurationNode& wp_tree = GetNode(et_tree, "team");
            /* Go through the nodes (waypoints) */
            TConfigurationNodeIterator itWaypt;
            for(itWaypt = itWaypt.begin(&wp_tree);
                itWaypt != itWaypt.end();
                ++itWaypt) {

                /* Get current node (waypoint) */
                TConfigurationNode& tWaypt = *itWaypt;
                /* Coordinate of waypoint */
                CVector2 coord;
                GetNodeAttribute(tWaypt, "coord", coord);
                waypoints.push(coord);
            }
            /* Get the newly created leader */
            std::ostringstream cEPId;
            cEPId.str("");
            cEPId << "L" << unPlacedLeaders;
            CEPuckEntity& cEPuck = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity(cEPId.str()));
            CLeader& cController = dynamic_cast<CLeader&>(cEPuck.GetControllableEntity().GetController());
            /* Set list of waypoints to leader */
            cController.SetWaypoints(waypoints);

            /* Update robot count */
            unPlacedLeaders += unLeaders;
            unPlacedRobots += unRobots;
        }
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::Reset() {
//    /* Zero the counters */
//    m_unCollectedFood = 0;
//    m_nEnergy = 0;
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
                 "chain" << std::endl;
//    /* Distribute uniformly the items in the environment */
//    for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
//       m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
//                         m_pcRNG->Uniform(m_cForagingArenaSideY));
//    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::Destroy() {
    /* Close the file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::PreStep() {

    UInt32 unFollowers1 = 0;
    UInt32 unFollowers2 = 0;
    UInt32 unChains = 0;
    std::unordered_map<std::string,CVector2> leaderPos;

    /* Get all the e-pucks */
    CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");

    for(CSpace::TMapPerType::iterator it = m_cEPucks.begin();
        it != m_cEPucks.end();
        ++it) {

        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        
        if(dynamic_cast<CFollower*>(&cEPuck.GetControllableEntity().GetController())) {
            /* If the e-puck is a FOLLOWER robot */
            CFollower& cController = dynamic_cast<CFollower&>(cEPuck.GetControllableEntity().GetController());

            /* Count how many e-pucks are in each state */
            if( cController.currentState == CFollower::RobotState::FOLLOWER ) {
                if( cController.GetTeamID() == 1 ) ++unFollowers1;
                else ++unFollowers2;
            } 
            else ++unChains;

        } else if(dynamic_cast<CLeader*>(&cEPuck.GetControllableEntity().GetController())) {
            /* If the e-puck is a LEADER robot */
            CLeader& cController = dynamic_cast<CLeader&>(cEPuck.GetControllableEntity().GetController());

            /* Get the position of the leader on the ground as a CVector2 */
            CVector2 cPos = CVector2(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                     cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            leaderPos[cEPuck.GetId()] = cPos;
        }
    }

//    /* Logic to pick and drop food items */
//    /*
//     * If a robot is in the nest, drop the food item
//     * If a robot is on a food item, pick it
//     * Each robot can carry only one food item per time
//     */
//    UInt32 unWalkingFBs = 0;
//    UInt32 unRestingFBs = 0;
//    /* Check whether a robot is on a food item */
//    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

//    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
//        it != m_cFootbots.end();
//        ++it) {
//       /* Get handle to foot-bot entity and controller */
//       CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
//       CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
//       /* Count how many foot-bots are in which state */
//       if(! cController.IsResting()) ++unWalkingFBs;
//       else ++unRestingFBs;
//       /* Get the position of the foot-bot on the ground as a CVector2 */
//       CVector2 cPos;
//       cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
//                cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
//       /* Get food data */
//       CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
//       /* The foot-bot has a food item */
//       if(sFoodData.HasFoodItem) {
//          /* Check whether the foot-bot is in the nest */
//          if(cPos.GetX() < -1.0f) {
//             /* Place a new food item on the ground */
//             m_cFoodPos[sFoodData.FoodItemIdx].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
//                                                   m_pcRNG->Uniform(m_cForagingArenaSideY));
//             /* Drop the food item */
//             sFoodData.HasFoodItem = false;
//             sFoodData.FoodItemIdx = 0;
//             ++sFoodData.TotalFoodItems;
//             /* Increase the energy and food count */
//             m_nEnergy += m_unEnergyPerFoodItem;
//             ++m_unCollectedFood;
//             /* The floor texture must be updated */
//             m_pcFloor->SetChanged();
//          }
//       }
//       else {
//          /* The foot-bot has no food item */
//          /* Check whether the foot-bot is out of the nest */
//          if(cPos.GetX() > -1.0f) {
//             /* Check whether the foot-bot is on a food item */
//             bool bDone = false;
//             for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
//                if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
//                   /* If so, we move that item out of sight */
//                   m_cFoodPos[i].Set(100.0f, 100.f);
//                   /* The foot-bot is now carrying an item */
//                   sFoodData.HasFoodItem = true;
//                   sFoodData.FoodItemIdx = i;
//                   /* The floor texture must be updated */
//                   m_pcFloor->SetChanged();
//                   /* We are done */
//                   bDone = true;
//                }
//             }
//          }
//       }
//    }

    /* Output stuff to file */
    m_cOutput << GetSpace().GetSimulationClock() << ","
              << leaderPos["L1"].GetX() << ","
              << leaderPos["L1"].GetY() << ","
              << unFollowers1 << ","
              << leaderPos["L2"].GetX() << ","
              << leaderPos["L2"].GetY() << ","
              << unFollowers2 << ","
              << unChains << std::endl;
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
            pcEP = new CEPuckEntity(cEPId.str(),
                                    HL_CONTROLLER,
                                    CVector3(),
                                    CQuaternion(),
                                    EP_RAB_RANGE,
                                    EP_RAB_DATA_SIZE,
                                    "");
            AddEntity(*pcEP);
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

REGISTER_LOOP_FUNCTIONS(CExperimentLoopFunctions, "experiment_loop_functions")
