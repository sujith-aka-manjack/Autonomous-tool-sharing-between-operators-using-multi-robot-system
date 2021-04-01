/**
 * @file <epuckpos/simulator/dynamics2d_epuckpos_model.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "dynamics2d_epuckpos_model.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_gripping.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real EPUCKPOS_MASS                = 0.4f;

   static const Real EPUCKPOS_RADIUS              = 0.035f;
   static const Real EPUCKPOS_INTERWHEEL_DISTANCE = 0.053f;
   static const Real EPUCKPOS_HEIGHT              = 0.086f;

   static const Real EPUCKPOS_MAX_FORCE           = 1.5f;
   static const Real EPUCKPOS_MAX_TORQUE          = 1.5f;

   enum EPUCKPOS_WHEELS {
      EPUCKPOS_LEFT_WHEEL = 0,
      EPUCKPOS_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CDynamics2DEPuckPosModel::CDynamics2DEPuckPosModel(CDynamics2DEngine& c_engine,
                                                CEPuckPosEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cEPuckPosEntity(c_entity),
      m_cWheeledEntity(m_cEPuckPosEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      EPUCKPOS_MAX_FORCE,
                      EPUCKPOS_MAX_TORQUE,
                      EPUCKPOS_INTERWHEEL_DISTANCE,
                      c_entity.GetConfigurationNode()),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Create the body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(EPUCKPOS_MASS,
                                  cpMomentForCircle(EPUCKPOS_MASS,
                                                    0.0f,
                                                    EPUCKPOS_RADIUS + EPUCKPOS_RADIUS,
                                                    cpvzero)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());
      /* Create the body shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpCircleShapeNew(ptBody,
                                          EPUCKPOS_RADIUS,
                                          cpvzero));
      ptShape->e = 0.0; // No elasticity
      ptShape->u = 0.7; // Lots of friction
      /* Constrain the actual base body to follow the diff steering control */
      m_cDiffSteering.AttachTo(ptBody);
      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, EPUCKPOS_HEIGHT);
   }

   /****************************************/
   /****************************************/

   CDynamics2DEPuckPosModel::~CDynamics2DEPuckPosModel() {
      m_cDiffSteering.Detach();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DEPuckPosModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DEPuckPosModel::UpdateFromEntityStatus() {
      /* Do we want to move? */
      if((m_fCurrentWheelVelocity[EPUCKPOS_LEFT_WHEEL] != 0.0f) ||
         (m_fCurrentWheelVelocity[EPUCKPOS_RIGHT_WHEEL] != 0.0f)) {
         m_cDiffSteering.SetWheelVelocity(m_fCurrentWheelVelocity[EPUCKPOS_LEFT_WHEEL],
                                          m_fCurrentWheelVelocity[EPUCKPOS_RIGHT_WHEEL]);
      }
      else {
         /* No, we don't want to move - zero all speeds */
         m_cDiffSteering.Reset();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CEPuckPosEntity, CDynamics2DEPuckPosModel);

   /****************************************/
   /****************************************/

}
