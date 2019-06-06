add_executable(test_load_urdf_model test_load_urdf_model.cpp)
add_executable(test_valkyrie_model test_valkyrie_model.cpp ${PROJECT_SOURCES})

target_link_libraries(test_load_urdf_model ${catkin_LIBRARIES})
target_link_libraries(test_valkyrie_model  ${catkin_LIBRARIES})