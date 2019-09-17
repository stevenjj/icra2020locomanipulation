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
