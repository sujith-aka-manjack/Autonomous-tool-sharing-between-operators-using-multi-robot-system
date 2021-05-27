/**
 * @file <circle_task/circle_task_entity.cpp>
 *
 * @author Genki Miyauchi - <g.miyauchi@sheffield.ac.uk>
 */

#include "circle_task_entity.h"
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

   /****************************************/
   /****************************************/

   CCircleTaskEntity::CCircleTaskEntity():
      CComposableEntity(nullptr) {}

   /****************************************/
   /****************************************/

   CCircleTaskEntity::CCircleTaskEntity(const std::string& str_id,
                                        const CVector2& c_position,
                                        Real f_radius,
                                        UInt32 un_demand,
                                        UInt32 un_min_robot_num,
                                        UInt32 un_max_robot_num) :
      CComposableEntity(nullptr, str_id) {}

   /****************************************/
   /****************************************/

   void CCircleTaskEntity::Init(TConfigurationNode& t_tree) {
      try {
         // /* Init parent */
         // CComposableEntity::Init(t_tree);
         // /* Parse XML to get the size */
         // GetNodeAttribute(t_tree, "size", m_cSize);
         // /* Parse XML to get the movable attribute */         
         // bool bMovable;
         // GetNodeAttribute(t_tree, "movable", bMovable);
         // if(bMovable) {
         //    /* Parse XML to get the mass */
         //    GetNodeAttribute(t_tree, "mass", m_fMass);
         // }
         // else {
         //    m_fMass = 0.0f;
         // }
         // /* Create embodied entity using parsed data */
         // m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         // AddComponent(*m_pcEmbodiedEntity);
         // m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         // m_pcEmbodiedEntity->SetMovable(bMovable);
         // /* Init LED equipped entity component */
         // m_pcLEDEquippedEntity = new CLEDEquippedEntity(this);
         // AddComponent(*m_pcLEDEquippedEntity);
         // if(NodeExists(t_tree, "leds")) {
         //    /* Create LED equipped entity
         //     * NOTE: the LEDs are not added to the medium yet
         //     */
         //    m_pcLEDEquippedEntity->Init(GetNode(t_tree, "leds"));
         //    /* Add the LEDs to the medium */
         //    std::string strMedium;
         //    GetNodeAttribute(GetNode(t_tree, "leds"), "medium", strMedium);
         //    m_pcLEDMedium = &CSimulator::GetInstance().GetMedium<CLEDMedium>(strMedium);
         //    m_pcLEDEquippedEntity->SetMedium(*m_pcLEDMedium);
         //    m_pcLEDEquippedEntity->Enable();
         // }
         // UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize box entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CCircleTaskEntity::Reset() {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CCircleTaskEntity,
                   "circle_task",
                   "Genki Miyauchi [g.miyauchi@sheffield.ac.uk]",
                   "1.0",
                   "A task with a circular area.",
                   "The circle task entity can be used to model a task with a circular area",
                   "Usable"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CCircleTaskEntity);

   /****************************************/
   /****************************************/

}