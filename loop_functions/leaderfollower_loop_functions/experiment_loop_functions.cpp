#include "experiment_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/leader/leader.h>
#include <controllers/follower/follower.h>

#include <unordered_map>

/****************************************/
/****************************************/

CExperimentLoopFunctions::CExperimentLoopFunctions() /* :
   m_cForagingArenaSideX(-0.9f, 1.7f),
   m_cForagingArenaSideY(-1.7f, 1.7f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unCollectedFood(0),
   m_nEnergy(0),
   m_unEnergyPerFoodItem(1),
   m_unEnergyPerWalkingRobot(1) */ {
}

/****************************************/
/****************************************/

void CExperimentLoopFunctions::Init(TConfigurationNode& t_node) {
    
    std::cout << "Init experiment loop function" << std::endl;

    try {
        TConfigurationNode& tForaging = GetNode(t_node, "experiment");
//       /* Get a pointer to the floor entity */
//       m_pcFloor = &GetSpace().GetFloorEntity();
//       /* Get the number of food items we want to be scattered from XML */
//       UInt32 unFoodItems;
//       GetNodeAttribute(tForaging, "items", unFoodItems);
//       /* Get the number of food items we want to be scattered from XML */
//       GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
//       m_fFoodSquareRadius *= m_fFoodSquareRadius;
//       /* Create a new RNG */
//       m_pcRNG = CRandom::CreateRNG("argos");
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
            CLeader & cController = dynamic_cast<CLeader&>(cEPuck.GetControllableEntity().GetController());

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

REGISTER_LOOP_FUNCTIONS(CExperimentLoopFunctions, "experiment_loop_functions")
