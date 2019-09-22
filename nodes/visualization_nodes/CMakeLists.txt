# VISUALIZE DOOR
add_executable(node_visualize_door node_visualize_door.cpp)
add_dependencies(node_visualize_door ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_visualize_door locomanipulation_library)

# VISUALIZE CART
add_executable(node_visualize_cart node_visualize_cart.cpp)
add_dependencies(node_visualize_cart ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_visualize_cart locomanipulation_library)

# VISUALIZE BAG
# add_executable(node_visualize_bag node_visualize_bag.cpp)
# add_dependencies(node_visualize_bag ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(node_visualize_bag locomanipulation_library)
