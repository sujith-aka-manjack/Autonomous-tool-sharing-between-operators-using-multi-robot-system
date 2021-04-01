/**
 * @file <epuckpos/simulator/dynamics2d_epuckpos_model.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef DYNAMICS2D_EPUCKPOS_MODEL_H
#define DYNAMICS2D_EPUCKPOS_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DEPuckPosModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <epuckpos/simulator/epuckpos_entity.h>

namespace argos {

   class CDynamics2DEPuckPosModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DEPuckPosModel(CDynamics2DEngine& c_engine,
                              CEPuckPosEntity& c_entity);
      virtual ~CDynamics2DEPuckPosModel();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      
   private:

      CEPuckPosEntity& m_cEPuckPosEntity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;

   };

}

#endif
