add_executable(node_viz_lm_contact_transition_space node_viz_lm_contact_transition_space.cpp ${PROJECT_SOURCES})
add_dependencies(node_viz_lm_contact_transition_space ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_viz_lm_contact_transition_space ${PROJECT_LIBRARIES})

add_executable(node_viz_lm_ee_space node_viz_lm_ee_space.cpp ${PROJECT_SOURCES})
add_dependencies(node_viz_lm_ee_space ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_viz_lm_ee_space ${PROJECT_LIBRARIES})

add_executable(node_viz_foot_reachability node_viz_foot_reachability.cpp ${PROJECT_SOURCES})
add_dependencies(node_viz_foot_reachability ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_viz_foot_reachability ${PROJECT_LIBRARIES})

add_executable(node_viz_hand_reachability node_viz_hand_reachability.cpp ${PROJECT_SOURCES})
add_dependencies(node_viz_hand_reachability ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_viz_hand_reachability ${PROJECT_LIBRARIES})

