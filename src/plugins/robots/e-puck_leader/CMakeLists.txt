add_library(argos3plugin_simulator_epuckleader SHARED
  simulator/dynamics2d_epuckleader_model.cpp
  simulator/dynamics2d_epuckleader_model.h
  simulator/epuckleader_entity.cpp
  simulator/epuckleader_entity.h
  simulator/qtopengl_epuckleader.cpp
  simulator/qtopengl_epuckleader.h)
target_link_libraries(argos3plugin_simulator_epuckleader
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media)
if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(argos3plugin_simulator_epuckleader
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_QTOPENGL_FOUND)
