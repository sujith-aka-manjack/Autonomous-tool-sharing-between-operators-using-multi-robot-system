add_library(test_possensor MODULE 
  test_possensor.h 
  test_possensor.cpp 
  SCT.h 
  SCT.cpp
  pid.h
  pid.cpp)
target_link_libraries(test_possensor
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
