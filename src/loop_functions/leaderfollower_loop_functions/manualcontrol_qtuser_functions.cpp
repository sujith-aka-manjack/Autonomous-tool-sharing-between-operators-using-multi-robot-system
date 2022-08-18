#include "manualcontrol_qtuser_functions.h"
#include <QKeyEvent>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <controllers/follower/follower.h>

/****************************************/
/****************************************/

static const Real DIRECTION_VECTOR_FACTOR = 10.;

/****************************************/
/****************************************/

CManualControlQTUserFunctions::CManualControlQTUserFunctions() :
   m_pcController(NULL),
   m_bSignal(false) {
   /* No key is pressed initially */
   m_punPressedKeys[DIRECTION_FORWARD]  = 0;
   m_punPressedKeys[DIRECTION_BACKWARD] = 0;
   m_punPressedKeys[DIRECTION_LEFT]     = 0;
   m_punPressedKeys[DIRECTION_RIGHT]    = 0;
   m_punPressedKeys[SIGNAL]             = 0;

   /* Register function to draw entity name */
   RegisterUserFunction<CManualControlQTUserFunctions,CEPuckEntity>(&CManualControlQTUserFunctions::Draw);
   RegisterUserFunction<CManualControlQTUserFunctions,CEPuckLeaderEntity>(&CManualControlQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::KeyPressed(QKeyEvent* pc_event) {
   /* Make sure that a controller was set */
   if(!m_pcController) {
      GetQTOpenGLWidget().KeyPressed(pc_event);
      return;
   }
   switch(pc_event->key()) {
      case Qt::Key_I:
         /* Forwards */
         m_punPressedKeys[DIRECTION_FORWARD] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_K:
         /* Backwards */
         m_punPressedKeys[DIRECTION_BACKWARD] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_J:
         /* Left */
         m_punPressedKeys[DIRECTION_LEFT] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_L:
         /* Right */
         m_punPressedKeys[DIRECTION_RIGHT] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_S:
         /* Toggle signal */
         m_punPressedKeys[SIGNAL] = 1;
         SetSignalFromKeyEvent();
         break;
      default:
         /* Unknown key */
         GetQTOpenGLWidget().KeyPressed(pc_event);
         break;
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::KeyReleased(QKeyEvent* pc_event) {
   /* Make sure that a controller was set */
   if(!m_pcController) {
      GetQTOpenGLWidget().KeyReleased(pc_event);
      return;
   }
   switch(pc_event->key()) {
      case Qt::Key_I:
         /* Forwards */
         m_punPressedKeys[DIRECTION_FORWARD] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_K:
         /* Backwards */
         m_punPressedKeys[DIRECTION_BACKWARD] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_J:
         /* Left */
         m_punPressedKeys[DIRECTION_LEFT] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_L:
         /* Right */
         m_punPressedKeys[DIRECTION_RIGHT] = 0;
         SetDirectionFromKeyEvent();
         break;
      default:
         /* Unknown key */
         GetQTOpenGLWidget().KeyReleased(pc_event);
         break;
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::EntitySelected(CEntity& c_entity) {
   /* Make sure the entity is an e-puck */
   CEPuckLeaderEntity* pcFB = dynamic_cast<CEPuckLeaderEntity*>(&c_entity);
   if(!pcFB) return;
   /* It's an e-puck_leader; extract its controller */
   m_pcController = dynamic_cast<CLeader*>(&pcFB->GetControllableEntity().GetController());
   /* Tell that e-puck that it is selected */
   m_pcController->Select();
   /* Reset key press information */
   m_punPressedKeys[DIRECTION_FORWARD]  = 0;
   m_punPressedKeys[DIRECTION_BACKWARD] = 0;
   m_punPressedKeys[DIRECTION_LEFT]     = 0;
   m_punPressedKeys[DIRECTION_RIGHT]    = 0;
   m_punPressedKeys[SIGNAL]             = 0;
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::EntityDeselected(CEntity& c_entity) {
   /* Make sure that a controller was set (should always be true...) */
   if(!m_pcController) return;
   /* Tell the e-puck that it is deselected */
   m_pcController->Deselect();
   /* Forget the controller */
   m_pcController = NULL;
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::SetDirectionFromKeyEvent() {
   /* Forward/backward direction factor (local robot X axis) */
   SInt32 FBDirection = 0;
   /* Left/right direction factor (local robot Y axis) */
   SInt32 LRDirection = 0;
   /* Calculate direction factor */
   if(m_punPressedKeys[DIRECTION_FORWARD])  ++FBDirection;
   if(m_punPressedKeys[DIRECTION_BACKWARD]) --FBDirection;
   if(m_punPressedKeys[DIRECTION_LEFT])     ++LRDirection;
   if(m_punPressedKeys[DIRECTION_RIGHT])    --LRDirection;
   /* Calculate direction */
   CVector2 cDir =
      DIRECTION_VECTOR_FACTOR *
      (CVector2(FBDirection, 0.0f) +
       CVector2(0.0f, LRDirection));
   /* Set direction */
   m_pcController->SetControlVector(cDir);
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::SetSignalFromKeyEvent() {
   m_bSignal = !m_bSignal;
   m_pcController->SetSignal(m_bSignal);
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::Draw(CEPuckEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the e-puck
    * See also the description in
    * $ argos3 -q e-puck
    */
   try {
      CFollower& cController = dynamic_cast<CFollower&>(c_entity.GetControllableEntity().GetController());
      // CFollower& cController = dynamic_cast<CFollower&>(cEPuck.GetControllableEntity().GetController());
      //std::string text = c_entity.GetId().c_str();
      std::string text;
      UInt8 temp = cController.GetRobotType();
      switch (temp)
      {
         case 1:
               text = "A" + cController.GetId().substr(1);
               break;
         case 2:
               text = "B" + cController.GetId().substr(1);
               break;
         case 3:
               text = "C" + cController.GetId().substr(1);
               break;
         case 4:
               text = "D" + cController.GetId().substr(1);
               break;
         case 5:
               text = "E" + cController.GetId().substr(1);
               break;
         case 6:
               text = "F" + cController.GetId().substr(1);
               break;
         case 7:
               text = "G" + cController.GetId().substr(1);
               break;
         case 8:
               text = "H" + cController.GetId().substr(1);
               break;
         case 9:
               text = "I" + cController.GetId().substr(1);
               break;
         case 10:
               text = "J" + cController.GetId().substr(1);
               break;
         
         default:
               break;
      }
      //std::string text = cController.GetId();

      /* For connector, draw the hop count to each team */
      if(cController.GetRobotState() == RobotState::CONNECTOR) {
         std::map<UInt8,HopMsg> hops = cController.GetHops();

         for(const auto& it : hops) {
            text += "(T" + std::to_string(it.first);
            if( !it.second.ID.empty() ) {
               text += "-" + it.second.ID;
            } else {
               text += "-__";
            }
            text += "-" + std::to_string(it.second.count) + ")";
         }
      }

      DrawText(CVector3(0.0, 0.0, 0.2),   // position
               text); // text
   } catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While casting robot as a follower", ex);
   } catch(const std::bad_cast& e) {
      std::cout << e.what() << " in Draw" << '\n';
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::Draw(CEPuckLeaderEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the e-puck_leader
    * See also the description in
    * $ argos3 -q e-puck_leader
    */
   QFont leaderFont("Helvetica [Cronyx]", 20, QFont::Bold);
   DrawText(CVector3(0.0, 0.0, 0.2),   // position
            c_entity.GetId().c_str(),
            CColor::BLACK,
            leaderFont); // text
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctions::DrawInWorld() {

   /* 
   * Draw Circle Tasks 
   */

   /* Get all the circle tasks */
   // CSpace::TMapPerType& m_cTasks = CSimulator::GetInstance().GetSpace().GetEntitiesByType("circle_task");
   
   CSpace::TMapPerType* m_cTasks;
   try {
      // m_cTasks = &CSimulator::GetInstance().GetSpace().GetEntitiesByType("circle_task");
      m_cTasks = &CSimulator::GetInstance().GetSpace().GetEntitiesByType("rectangle_task");
   } catch(CARGoSException& ex) {
      std::cout << "No circle task found in argos file (DrawInWorld)" << std::endl;
   }

   if( !m_cTasks->empty() ) {
      for(CSpace::TMapPerType::iterator it = m_cTasks->begin();
       it != m_cTasks->end();
       ++it) {

         /* Get handle to the circle task entity */
         // CCircleTaskEntity& cTask = *any_cast<CCircleTaskEntity*>(it->second);
         CRectangleTaskEntity& cTask = *any_cast<CRectangleTaskEntity*>(it->second);

         /* Draw circle task */
         CVector2 pos = cTask.GetPosition();
         UInt32 demand = cTask.GetDemand();

         // std::cout << cTask.GetWidth() << " " << cTask.GetHeight() << std::endl;

         std::vector<CVector2> points = { CVector2(-0.5*cTask.GetWidthX(),0.5*cTask.GetWidthY()),
                                          CVector2(0.5*cTask.GetWidthX(),0.5*cTask.GetWidthY()),
                                          CVector2(0.5*cTask.GetWidthX(),-0.5*cTask.GetWidthY()),
                                          CVector2(-0.5*cTask.GetWidthX(),-0.5*cTask.GetWidthY())};

         // for(const auto& pt : points) {
         //    std::cout << pt << std::endl;
         // }

         if(demand > 0) {
            // DrawCircle(CVector3(pos.GetX(), pos.GetY(), 0.001),
            //          CQuaternion(),
            //          cTask.GetRadius(),
            //          CColor(255U, 128U, 128U, 255U),
            //          true,
            //          40U);
            DrawPolygon(CVector3(pos.GetX(), pos.GetY(), 0.001),
                        CQuaternion(),
                        points,
                        CColor(255U, 128U, 128U, 255U));
         } else {
            // DrawCircle(CVector3(pos.GetX(), pos.GetY(), 0.001),
            //          CQuaternion(),
            //          cTask.GetRadius(),
            //          CColor(128U, 255U, 128U, 255U),
            //          true,
            //          40U);
            // std::vector<CVector2> points = {CVector2(-0.5,0.5),CVector2(0.5,0.5),CVector2(0.5,-0.5),CVector2(-0.5,-0.5)};
            DrawPolygon(CVector3(pos.GetX(), pos.GetY(), 0.001),
                        CQuaternion(),
                        points,
                        CColor(128U, 255U, 128U, 255U));
         }
         
         /* Draw task info */
         std::ostringstream cText;
         cText.str("");
         // cText << ceil(cTask.GetDemand() / 10);
         cText << cTask.GetDemand();
         QFont taskFont("Helvetica [Cronyx]", 30, QFont::Bold);
         // DrawText(CVector3(pos.GetX(), pos.GetY()+cTask.GetRadius()/2, 0.01),
         //          cText.str(),
         //          CColor::BLACK,
         //          taskFont);
         DrawText(CVector3(pos.GetX()-0.3, pos.GetY()+0.1, 0.01),
                  cText.str(),
                  CColor::BLACK,
                  taskFont);

         QFont numFont("Helvetica [Cronyx]", 15);
         cText.str("");
         // UInt32 temp1[RType];
         // std::fill_n(temp1, RType, 0);
         // for(int i=0; i<RType; ++i)
         //    temp1[i]= cTask.GetCurrentRobotNum(i);
         // //UInt32* temp1 =  cTask.GetCurrentRobotNum();
         // UInt32* temp2 =  cTask.GetMinRobotNum();
         // // static UInt32 val1[RType];
         // // static UInt32 val2[RType];
         // // for (int i=0; i<RType; ++i){
         // //       val1[i] = temp1[i];
         // //       val2[i] = temp2[i];
         // // }
         // for(int i=0; i<RType; ++i)
         //    // cText << val1[1] << " / " << val2[1] << ", ";
         //   cText << temp1[i] << " / " << *(cTask.GetMinRobotNum()+i) << ", ";
         Real x_pos = pos.GetX();
         Real y_pos = pos.GetY();
         for(int i=0; i<RType; ++i){     //Only for odd no of robot types
            cText.str("");
            if(i%2==0){
               cText << cTask.GetCurrentRobotNum(i) << " / " << *(cTask.GetMinRobotNum()+i) << ",   ";
                DrawText(CVector3(x_pos-0.3, y_pos-(0.05*1.5*(i+1)), 0.01),
                     cText.str(),
                     CColor::WHITE,
                     numFont);
            }
            else{
               cText << cTask.GetCurrentRobotNum(i) << " / " << *(cTask.GetMinRobotNum()+i+1);
                DrawText(CVector3(x_pos+0.1, y_pos-(0.05*1.5*(i)), 0.01),
                     cText.str(),
                     CColor::WHITE,
                     numFont);
            }
         //    DrawText(CVector3(pos.GetX()-0.3, pos.GetY()-(0.05*1.5*(i+1)), 0.01),
         //             cText.str(),
         //             CColor::BLACK,
         //             numFont);
         }
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CManualControlQTUserFunctions, "manualcontrol_qtuser_functions")
