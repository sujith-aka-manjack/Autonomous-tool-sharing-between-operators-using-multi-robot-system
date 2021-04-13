add_library(follower2 MODULE 
  follower2.h 
  follower2.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(follower2
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
