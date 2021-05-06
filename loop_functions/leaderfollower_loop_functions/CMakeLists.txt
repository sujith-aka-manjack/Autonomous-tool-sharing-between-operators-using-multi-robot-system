add_library(leaderfollower_loop_functions MODULE 
  manualcontrol_qtuser_functions.h
  manualcontrol_qtuser_functions.cpp)

target_link_libraries(leaderfollower_loop_functions
  leader
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
