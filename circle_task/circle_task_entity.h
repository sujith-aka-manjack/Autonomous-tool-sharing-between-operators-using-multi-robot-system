/**
 * @file <circle_task/circle_task_entity.h>
 *
 * @author Genki Miyauchi - <g.miyauchi@sheffield.ac.uk>
 */

#ifndef CIRCLE_TASK_ENTITY_H
#define CIRCLE_TASK_ENTITY_H

namespace argos {
   class CCircleTaskEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CCircleTaskEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      CCircleTaskEntity();

      CCircleTaskEntity(const std::string& str_id,
                        const CVector2& c_position,
                        Real f_radius,
                        UInt32 un_demand = 0,
                        UInt32 un_min_robot_num = 1,
                        UInt32 un_max_robot_num = 4294967295);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();

      virtual std::string GetTypeDescription() const {
         return "circle_task";
      }

   private:

      CVector2            m_cPos;
      Real                m_fRadius;
      UInt32              m_unDemand;
      UInt32              m_unMinRobotNum;
      UInt32              m_unMaxRobotNum;

   };

}

#endif