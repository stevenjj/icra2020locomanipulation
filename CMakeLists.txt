cmake_minimum_required(VERSION 2.8.3)
project(avatar_locomanipulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS eigenpy pinocchio urdf hpp-fcl
	roscpp
	sensor_msgs
	tf
	rviz)

add_definitions(-Wno-deprecated-declarations)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

find_package(Eigen3 3.2 REQUIRED)
#message("eigen version:" ${EIGEN3_VERSION})
find_package(Boost 1.58 REQUIRED COMPONENTS system)
find_package(YamlCpp 0.5.2 REQUIRED) # Using a custom FindYamlcpp.cmake

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS eigenpy pinocchio urdf roscpp sensor_msgs tf rviz
)

###########
## Build ##
###########
SET (THIS_PACKAGE_PATH "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/cmake/config.h.cmake ${PROJECT_SOURCE_DIR}/include/Configuration.h)

## Specify additional locations of header files
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)

SET (MODEL_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/models/valkyrie_model.cpp)

SET (DATA_TYPES_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/data_types/footstep.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/data_types/trajectory_euclidean.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/data_types/trajectory_orientation.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/data_types/trajectory_SE3.cpp)


SET (WALKING_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/walking/walking_pattern_generator.cpp)

SET (HELPER_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/hermite_curve.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/hermite_curve_vec.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/hermite_quaternion_curve.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/rkf45.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/convex_hull.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/pseudo_inverse.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/param_handler.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/yaml_data_saver.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/gmm_fit.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/helpers/orientation_utils.cpp
)

SET (ROS_BRIDGE_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/bridge/val_rviz_translator.cpp)

SET (FEASIBILITY_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/feasibility/feasibility.cpp)

SET (TASK_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task_com.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task_6dpose.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task_6dpose_wrt_midfoot.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task_6dcontact_normal.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task_4dcontact_normal.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task_3dorientation.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task_joint_config.cpp
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/tasks/task_stack.cpp)

SET (COLLISION_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/collision_module/collision_class.cpp)

SET (IK_MODULE_SOURCES
	${PROJECT_SOURCE_DIR}/src/avatar_locomanipulation/ik_module/ik_module.cpp)

SET(PROJECT_SOURCES
	${MODEL_SOURCES}
	${DATA_TYPES_SOURCES}
	${WALKING_SOURCES}
	${HELPER_SOURCES}
	${ROS_BRIDGE_SOURCES}
	${FEASIBILITY_SOURCES}
	${TASK_SOURCES}
	${COLLISION_SOURCES}
	${IK_MODULE_SOURCES})

# Grab all the libraries
SET(PROJECT_LIBRARIES
	${catkin_LIBRARIES}
	${YAML_CPP_LIBRARIES})

add_subdirectory(test)
