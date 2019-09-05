#add_executable(test_astar_planner test_astar_planner.cpp ${PROJECT_SOURCES})
add_executable(test_planner_with_trajectories test_planner_with_trajectories.cpp ${PROJECT_SOURCES})

#target_link_libraries(test_astar_planner ${PROJECT_LIBRARIES})
target_link_libraries(test_planner_with_trajectories ${PROJECT_LIBRARIES})

# add_dependencies(test_astar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(test_planner_with_trajectories ${${PROJECT_NAME}_EXPORTED_TARGETS})
