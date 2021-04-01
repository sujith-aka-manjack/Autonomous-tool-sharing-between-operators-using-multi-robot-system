add_library(argos3plugin_simulator_epuckpos SHARED
  simulator/dynamics2d_epuckpos_model.cpp
  simulator/dynamics2d_epuckpos_model.h
  simulator/epuckpos_entity.cpp
  simulator/epuckpos_entity.h
  simulator/qtopengl_epuckpos.cpp
  simulator/qtopengl_epuckpos.h)
target_link_libraries(argos3plugin_simulator_epuckpos
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_simulator_epuckpos
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_COMPILE_QTOPENGL)
