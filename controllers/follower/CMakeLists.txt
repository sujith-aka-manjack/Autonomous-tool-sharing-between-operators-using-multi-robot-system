add_library(follower SHARED 
  follower.h 
  follower.cpp 
  SCT.h 
  SCT.cpp
  pid.h
  pid.cpp)
target_link_libraries(follower
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
