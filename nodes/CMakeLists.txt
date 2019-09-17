# add_executable(classifier_client_node classifier_client_node.cpp ${PROJECT_SOURCES})
# add_dependencies(classifier_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(classifier_client_node ${PROJECT_LIBRARIES})

# DOOR OPEN
add_executable(planner_door_open_node planner_door_open_node.cpp ${PROJECT_SOURCES})
add_dependencies(planner_door_open_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(planner_door_open_node ${PROJECT_LIBRARIES})
# DOOR OPEN Neural Net
add_executable(planner_door_open_neural_net_node planner_door_open_neural_net_node.cpp ${PROJECT_SOURCES})
add_dependencies(planner_door_open_neural_net_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(planner_door_open_neural_net_node ${PROJECT_LIBRARIES})
# Cart Push
add_executable(planner_cart_push_node planner_cart_push_node.cpp ${PROJECT_SOURCES})
add_dependencies(planner_cart_push_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(planner_cart_push_node ${PROJECT_LIBRARIES})
# Cart Push Neural Net
add_executable(planner_cart_push_neural_net_node planner_cart_push_neural_net_node.cpp ${PROJECT_SOURCES})
add_dependencies(planner_cart_push_neural_net_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(planner_cart_push_neural_net_node ${PROJECT_LIBRARIES})

# VISUALIZE DOOR
add_executable(node_visualize_door node_visualize_door.cpp ${PROJECT_SOURCES})
add_dependencies(node_visualize_door ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_visualize_door ${PROJECT_LIBRARIES})

# VISUALIZE CART
add_executable(node_visualize_cart node_visualize_cart.cpp ${PROJECT_SOURCES})
add_dependencies(node_visualize_cart ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_visualize_cart ${PROJECT_LIBRARIES})

# VISUALIZE BAG
# add_executable(node_visualize_bag node_visualize_bag.cpp ${PROJECT_SOURCES})
# add_dependencies(node_visualize_bag ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(node_visualize_bag ${PROJECT_LIBRARIES})

# Locomanipulability Visualization
# Contact Transition Space
add_executable(node_transition_space_visualization node_transition_space_visualization.cpp ${PROJECT_SOURCES})
add_dependencies(node_transition_space_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_transition_space_visualization ${PROJECT_LIBRARIES})