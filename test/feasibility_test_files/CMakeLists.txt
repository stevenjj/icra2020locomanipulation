# add_executable(test_feasibility_data_generation test_feasibility_data_generation.cpp ${PROJECT_SOURCES})
add_executable(test_generate_data test_generate_data.cpp ${PROJECT_SOURCES})
add_executable(test_feasibility_data_playback test_feasibility_data_playback.cpp ${PROJECT_SOURCES})

# target_link_libraries(test_feasibility_data_generation ${PROJECT_LIBRARIES})
target_link_libraries(test_generate_data ${PROJECT_LIBRARIES})
target_link_libraries(test_feasibility_data_playback ${PROJECT_LIBRARIES})
