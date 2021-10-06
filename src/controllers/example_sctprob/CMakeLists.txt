add_library(example_sctprob MODULE 
  example_sctprob.h 
  example_sctprob.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(example_sctprob
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
