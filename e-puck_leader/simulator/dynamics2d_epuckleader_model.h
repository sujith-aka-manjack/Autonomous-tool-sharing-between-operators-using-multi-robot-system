/**
 * @file <e-puck_leader/simulator/dynamics2d_epuckleader_model.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef DYNAMICS2D_EPUCKLEADER_MODEL_H
#define DYNAMICS2D_EPUCKLEADER_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DEPuckLeaderModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <e-puck_leader/simulator/epuckleader_entity.h>

namespace argos {

   class CDynamics2DEPuckLeaderModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DEPuckLeaderModel(CDynamics2DEngine& c_engine,
                              CEPuckLeaderEntity& c_entity);
      virtual ~CDynamics2DEPuckLeaderModel();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      
   private:

      CEPuckLeaderEntity& m_cEPuckLeaderEntity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;

   };

}

#endif
