add_library(example_sct MODULE 
  example_sct.h 
  example_sct.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(example_sct
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
