#
# e-puck_leader headers
#
# argos3/plugins/robots/e-puck_leader/control_interface
# set(ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCKLEADER_CONTROLINTERFACE
#   control_interface/ci_epuckleader_proximity_sensor.h)

# argos3/plugins/robots/e-puck_leader/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCKLEADER_SIMULATOR
    simulator/dynamics2d_epuckleader_model.h
    simulator/epuckleader_entity.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# e-puck_leader sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCKLEADER
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCKLEADER_CONTROLINTERFACE})
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCKLEADER
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCKLEADER}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCKLEADER_SIMULATOR}
    simulator/dynamics2d_epuckleader_model.cpp
    simulator/epuckleader_entity.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  if(ARGOS_QTOPENGL_FOUND)
    include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCKLEADER_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCKLEADER_SIMULATOR}
      simulator/qtopengl_epuckleader.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCKLEADER
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCKLEADER}
      simulator/qtopengl_epuckleader.h
      simulator/qtopengl_epuckleader.cpp)
  endif(ARGOS_QTOPENGL_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create e-puck_leader plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_epuckleader SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCKLEADER})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_epuckleader
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d)
if(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_QTOPENGL_FOUND)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_epuckleader argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_QTOPENGL_FOUND)

#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCKLEADER_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/e-puck_leader/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCKLEADER_SIMULATOR}      DESTINATION include/argos3/plugins/robots/e-puck_leader/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_epuckleader
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
