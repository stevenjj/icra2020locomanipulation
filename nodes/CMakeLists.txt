# add_executable(classifier_client_node classifier_client_node.cpp ${PROJECT_SOURCES})
# add_dependencies(classifier_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(classifier_client_node ${PROJECT_LIBRARIES})

# add_executable(node_visualize_door node_visualize_door.cpp ${PROJECT_SOURCES})
# add_dependencies(node_visualize_door ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(node_visualize_door ${PROJECT_LIBRARIES})

# add_executable(planner_door_open_node planner_door_open_node.cpp ${PROJECT_SOURCES})
# add_dependencies(planner_door_open_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(planner_door_open_node ${PROJECT_LIBRARIES})

add_executable(node_visualize_bag node_visualize_bag.cpp ${PROJECT_SOURCES})
add_dependencies(node_visualize_bag ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_visualize_bag ${PROJECT_LIBRARIES})


