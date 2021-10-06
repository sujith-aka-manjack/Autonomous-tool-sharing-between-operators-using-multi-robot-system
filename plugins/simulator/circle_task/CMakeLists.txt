add_library(argos3plugin_simulator_circletask SHARED
  circle_task_entity.cpp
  circle_task_entity.h
)
target_link_libraries(argos3plugin_simulator_circletask
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities)
if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(argos3plugin_simulator_circletask
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_QTOPENGL_FOUND)