include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})

add_library(leaderfollower_loop_functions MODULE 
  manualcontrol_qtuser_functions.h
  manualcontrol_qtuser_functions.cpp
  experiment_loop_functions.h
  experiment_loop_functions.cpp)

target_link_libraries(leaderfollower_loop_functions
  leader
  follower
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_epuckleader
  argos3plugin_simulator_qtopengl
  yaml-cpp
  ${ARGOS_QTOPENGL_LIBRARIES})
