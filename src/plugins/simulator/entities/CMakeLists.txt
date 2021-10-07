#
# Headers
#
# argos3/plugins/simulator/entities/
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_TASK_ENTITIES
  circle_task_entity.h
  rectangle_task_entity.h)

#
# Source files
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_TASK_ENTITIES
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_TASK_ENTITIES}
  circle_task_entity.cpp
  rectangle_task_entity.cpp)

#
# Create entity plugin library
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_task_entities SHARED ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_TASK_ENTITIES})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_task_entities argos3plugin_${ARGOS_BUILD_FOR}_media)

#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

#
# Core library installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_TASK_ENTITIES} DESTINATION include/argos3/plugins/simulator/entities/)
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_task_entities
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)