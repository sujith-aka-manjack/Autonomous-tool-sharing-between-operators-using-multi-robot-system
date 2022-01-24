/**
 * @file <rectangle_task/rectangle_task_entity.h>
 *
 * @author Genki Miyauchi - <g.miyauchi@sheffield.ac.uk>
 */

#ifndef RECTANGLE_TASK_ENTITY_H
#define RECTANGLE_TASK_ENTITY_H

namespace argos {
   class CRectangleTaskEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CRectangleTaskEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      CRectangleTaskEntity();

      CRectangleTaskEntity(const std::string& str_id,
                        const CVector2& c_position,
                        Real width,
                        Real height,
                        UInt32 un_demand = 0,
                        UInt32 un_min_robot_num = 1,
                        UInt32 un_max_robot_num = 4294967295);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();

      virtual CVector2 GetPosition() const {
         return m_cPos;
      }

      virtual Real GetWidth() const {
         return m_fWidth;
      }

      virtual Real GetHeight() const {
         return m_fHeight;
      }

      virtual UInt32 GetDemand() const {
         return m_unDemand;
      }

      virtual UInt32 GetInitDemand() const {
         return m_unInitDemand;
      }

      virtual void SetDemand(UInt32 un_demand) {
         m_unDemand = un_demand;
      }

      virtual UInt32 GetMinRobotNum() const {
         return m_unMinRobotNum;
      }

      virtual UInt32 GetMaxRobotNum() const {
         return m_unMaxRobotNum;
      }

      virtual UInt32 GetCurrentRobotNum() const {
         if(m_unDemand > 0)
            return m_unCurrentRobotNum;
         else
            return 0;
      }

      virtual void SetCurrentRobotNum(UInt32 un_robot_num) {
         m_unCurrentRobotNum = un_robot_num;
      }

      virtual bool InArea(const CVector2& pos);

      virtual std::string GetTypeDescription() const {
         return "rectangle_task";
      }

   private:

      CVector2            m_cPos;
      Real                m_fRadius;
      Real                m_fWidth;
      Real                m_fHeight;
      UInt32              m_unDemand;
      UInt32              m_unInitDemand;
      UInt32              m_unMinRobotNum;
      UInt32              m_unMaxRobotNum;
      UInt32              m_unCurrentRobotNum;

   };

}

#endif