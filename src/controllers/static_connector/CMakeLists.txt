add_library(static_connector SHARED 
  static_connector.h 
  static_connector.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(static_connector
  utility
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
