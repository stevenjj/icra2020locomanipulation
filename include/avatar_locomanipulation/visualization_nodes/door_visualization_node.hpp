#ifndef ALM_DOOR_VISUALIZATION_H
#define ALM_DOOR_VISUALIZATION_H

// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <iostream>
#include <fstream>

#include <string>

#include <math.h>

class DoorVisualizationNode{
public:
	double s_current; // curent s for determining door angle
	double radius; // door radius
	double door_open_angle; // angle when door is fully open
	double handle_height; // height of handle (assumed 1/2 of door size)
	double pose_spacing; // spacing b/w each wp
	Eigen::Quaternion<double> q_init; // initial orientation for wp1
	Eigen::Vector3d wp1_pos; // initial position for wp1

	DoorVisualizationNode();

	DoorVisualizationNode(int & s_begin);

	~DoorVisualizationNode();

	void getParameters();

	void getVizInformation(visualization_msgs::Marker & door_msg, tf::Transform & tf_world_wp1, tf::Transform & tf_wp1_hinge);


};


#endif