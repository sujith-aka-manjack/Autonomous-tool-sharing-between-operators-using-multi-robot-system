add_library(example_sct_yaml MODULE 
  example_sct_yaml.h 
  example_sct_yaml.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(example_sct_yaml
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
