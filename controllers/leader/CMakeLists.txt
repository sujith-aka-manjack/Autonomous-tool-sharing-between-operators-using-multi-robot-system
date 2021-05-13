add_library(leader SHARED 
  leader.h 
  leader.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(leader
  utility
  argos3core_simulator
  argos3plugin_simulator_epuckleader
  argos3plugin_simulator_genericrobot)
