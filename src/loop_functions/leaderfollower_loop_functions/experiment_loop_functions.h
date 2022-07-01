#ifndef EXPERIMENT_LOOP_FUNCTIONS_H
#define EXPERIMENT_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

#include <unordered_map>

using namespace argos;

class CExperimentLoopFunctions : public CLoopFunctions {

public:

   CExperimentLoopFunctions();
   virtual ~CExperimentLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   virtual void PostStep();
   virtual bool IsLogging();
   virtual std::string GetCommandFilePath();
   virtual UInt32 GetCurrentPoints();

private:

   TConfigurationNode config;
   std::vector<std::string> m_vecEntityID;

   std::vector<CVector2> m_vecWaypointPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;
   bool m_bTaskExists;
   bool m_bTaskComplete;
   UInt32 m_unNextTaskId;
   UInt32 m_unTotalTasks;
   UInt32 m_unTaskDemand;
   UInt32 m_unPointsObtained;
   // std::vector<std::unordered_map<std::string,UInt32>> m_vecTaskDemand;
   std::map<UInt32, std::map<std::string, Real>> m_vecTaskPos;

   CRange<Real> cArenaSideSplitX[4] = {
                                       CRange<Real>(0.5f, 1.45f), // TEMPORARY: hard coded arena size
                                       CRange<Real>(0.5f, 1.45f),
                                       CRange<Real>(-1.45f, -0.5f),
                                       CRange<Real>(-1.45f, -0.5f),
                                      };
   CRange<Real> cArenaSideSplitY[4] = {
                                       CRange<Real>(-1.45f, -0.5f), // TEMPORARY: hard coded arena size
                                       CRange<Real>(0.5f, 1.45f),
                                       CRange<Real>(-1.45f, -0.5f),
                                       CRange<Real>(0.5f, 1.45f),
                                       };
   CRange<Real> cArenaSideX = CRange<Real>(-1.45f, 1.45f);
   CRange<Real> cArenaSideY = CRange<Real>(-1.45f, 1.45f);

   /* Output file */
   bool m_bLogging;
   std::string m_strOutput;
   std::string m_strDirPath;
   std::string m_strBinaryFilePath;
   std::string m_strSummaryFilePath;
   std::string m_strCommandFilePath;
   std::fstream m_cOutput;

   /* Frame Grabbing */
   bool m_bFrameGrabbing;
   UInt32 m_unCameraIndex;

   /* Init logging */
   void InitLogging();

   /* Init robots */
   void InitRobots();

   /* Init tasks */
   void InitTasks();

   /* Distribute a leader-robot team */
   void PlaceCluster(const CVector2& c_center,
                     UInt32 un_leaders,
                     UInt32 un_robots,
                     Real f_density,
                     UInt32 un_leader_id_start,
                     UInt32 un_robot_id_start);

   void PlaceCustomPosition(const CVector2& c_center,
                            std::string str_type,
                            UInt32 un_leader_id_start,
                            UInt32 un_robot_id_start);

   /* Place a task */
   void PlaceTask(const CVector2& c_center,
                  Real f_radius,
                  UInt32 un_demand,
                  UInt32 un_min_robot_num,
                  UInt32 un_max_robot_num,
                  UInt32 un_task_id_start);

   /* Place a task */
   void PlaceRectangleTask(const CVector2& c_center,
                  Real f_width_x,
                  Real f_width_y,
                  Real f_height,
                  UInt32 un_demand,
                  UInt32 un_min_robot_num,
                  UInt32 un_max_robot_num,
                  UInt32 un_task_id_start);
};

#endif
