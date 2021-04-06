add_library(leader MODULE 
  leader.h 
  leader.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(leader
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
