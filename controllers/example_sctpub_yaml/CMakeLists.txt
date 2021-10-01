add_library(example_sctpub_yaml MODULE 
  example_sctpub_yaml.h 
  example_sctpub_yaml.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(example_sctpub_yaml
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot
  yaml-cpp)
