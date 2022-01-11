add_library(utility SHARED
    team_color.h
    pid.h
    pid.cpp
    sct.h
    sct.cpp)
target_link_libraries(utility
    yaml-cpp)