#ifndef EXPERIMENT_LOOP_FUNCTIONS_H
#define EXPERIMENT_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
// #include <argos3/core/simulator/entity/floor_entity.h>
// #include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CExperimentLoopFunctions : public CLoopFunctions {

public:

   CExperimentLoopFunctions();
   virtual ~CExperimentLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
//    virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();

private:

//    Real m_fFoodSquareRadius;
//    CRange<Real> m_cExperimentArenaSideX, m_cExperimentArenaSideY;
//    std::vector<CVector2> m_cFoodPos;
//    CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   /* Output file */
   std::string m_strOutput;
   std::ofstream m_cOutput;

//    UInt32 m_unCollectedFood;
//    SInt64 m_nEnergy;
//    UInt32 m_unEnergyPerFoodItem;
//    UInt32 m_unEnergyPerWalkingRobot;

   /* Distribute a leader-robot team */
   void PlaceCluster(const CVector2& c_center,
                     UInt32 un_leaders,
                     UInt32 un_robots,
                     Real f_density,
                     UInt32 un_leader_id_start,
                     UInt32 un_robot_id_start);
};

#endif
