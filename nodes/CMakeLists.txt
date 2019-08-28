add_executable(classifier_client_node classifier_client_node.cpp ${PROJECT_SOURCES})
add_dependencies(classifier_client_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(classifier_client_node ${PROJECT_LIBRARIES})
