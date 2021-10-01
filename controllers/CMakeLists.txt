include_directories(${CMAKE_SOURCE_DIR}/controllers)

# add_subdirectory(example_sct)
# add_subdirectory(example_sctprob)
# add_subdirectory(example_sctpub)
# add_subdirectory(example_sctprobpub)
add_subdirectory(example_sct_yaml)
add_subdirectory(example_sctpub_yaml)

add_subdirectory(leader)
add_subdirectory(follower)
add_subdirectory(static_connector)
