add_library(utility SHARED
    team_color.h
    pid.h
    pid.cpp
    sct.h
    sct.cpp
    robot_message.h)
target_link_libraries(utility
    yaml-cpp)