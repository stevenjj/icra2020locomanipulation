add_executable(test_interpolation_module test_interpolation_module.cpp ${PROJECT_SOURCES})
# add_executable(test_visualize_door_waypoints test_visualize_door_waypoints.cpp ${PROJECT_SOURCES})
# add_executable(test_visualize_panel_waypoints test_visualize_panel_waypoints.cpp ${PROJECT_SOURCES})

target_link_libraries(test_interpolation_module ${PROJECT_LIBRARIES})
# target_link_libraries(test_visualize_door_waypoints ${PROJECT_LIBRARIES})
# target_link_libraries(test_visualize_panel_waypoints ${PROJECT_LIBRARIES})