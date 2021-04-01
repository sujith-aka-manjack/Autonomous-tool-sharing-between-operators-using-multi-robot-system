add_library(example_sctprobpub MODULE 
  example_sctprobpub.h 
  example_sctprobpub.cpp 
  SCT.h 
  SCT.cpp)
target_link_libraries(example_sctprobpub
  argos3core_simulator
  argos3plugin_simulator_epuckpos
  argos3plugin_simulator_genericrobot)
