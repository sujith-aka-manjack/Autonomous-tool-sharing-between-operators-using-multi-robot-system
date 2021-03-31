add_library(example_sctpub MODULE 
  example_sctpub.h 
  example_sctpub.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(example_sctpub
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
