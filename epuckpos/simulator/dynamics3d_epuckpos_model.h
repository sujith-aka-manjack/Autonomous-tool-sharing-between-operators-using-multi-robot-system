/**
 * @file <epuckpos/simulator/dynamics3d_epuckpos_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_EPUCKPOS_MODEL_H
#define DYNAMICS3D_EPUCKPOS_MODEL_H

namespace argos {
   class CDynamics3DEPuckPosModel;
   class CEPuckPosEntity;
   class CWheeledEntity;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_multi_body_object_model.h>

namespace argos {

   class CDynamics3DEPuckPosModel : public CDynamics3DMultiBodyObjectModel {

   public:     

      CDynamics3DEPuckPosModel(CDynamics3DEngine& c_engine,
                            CEPuckPosEntity& c_epuckpos);

      virtual ~CDynamics3DEPuckPosModel() {}

      virtual void Reset();

      virtual void CalculateBoundingBox();

      virtual void UpdateEntityStatus();

      virtual void UpdateFromEntityStatus();

      virtual void AddToWorld(btMultiBodyDynamicsWorld& c_world);

      virtual void RemoveFromWorld(btMultiBodyDynamicsWorld& c_world);

   private:
      /* joint constraints */
      std::unique_ptr<btMultiBodyJointMotor> m_ptrLeftMotor;
      std::unique_ptr<btMultiBodyJointMotor> m_ptrRightMotor;
      /* links */
      std::shared_ptr<CBase> m_ptrBody;
      std::shared_ptr<CLink> m_ptrLeftWheel;
      std::shared_ptr<CLink> m_ptrRightWheel;
      /* entities */
      CWheeledEntity& m_cWheeledEntity;
      /* lower base data */
      static const btVector3 m_cBodyHalfExtents;
      static const btScalar m_fBodyMass;
      static const btTransform m_cBodyOffset;
      static const btTransform m_cBodyGeometricOffset;
      /* wheel data */
      static const btVector3 m_cWheelHalfExtents;
      static const btScalar m_fWheelMass;
      static const btTransform m_cWheelGeometricOffset;
      static const btTransform m_cLeftWheelOffset;
      static const btTransform m_cRightWheelOffset;
      static const btVector3 m_cBodyToRightWheelJointOffset;
      static const btVector3 m_cRightWheelToBodyJointOffset;
      static const btQuaternion m_cBodyToRightWheelJointRotation;
      static const btVector3 m_cBodyToLeftWheelJointOffset;
      static const btVector3 m_cLeftWheelToBodyJointOffset;
      static const btQuaternion m_cBodyToLeftWheelJointRotation;
      static const btScalar m_fWheelMotorMaxImpulse;
      static const btScalar m_fWheelFriction;
   };
}

#endif
