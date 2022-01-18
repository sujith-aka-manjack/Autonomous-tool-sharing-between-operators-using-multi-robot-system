add_library(follower SHARED 
  follower.h 
  follower.cpp)
target_link_libraries(follower
  utility
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
