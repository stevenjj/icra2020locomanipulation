#ifndef ALM_CART_VISUALIZATION_H
#define ALM_CART_VISUALIZATION_H

// Package Path Definition
#include <Configuration.h>


// Import ROS and Rviz visualization
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
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
#include <std_msgs/Float64.h>

class CartVisualizationNode{
public:
	ros::NodeHandle* nh;

	ros::Subscriber s_sub;

	double s_current; // curent s for determining bag position

	double radius; // inner turn radius

	double hand_distance; // linear distance b/w hands/sets of wps

	double rotation_angle; // angle to turn cart

	double linear_length; // Distance to be pushed in straight line

	Eigen::Vector3d fixed_frame_pos;

	double pose_spacing; // spacing b/w each wp

	Eigen::Quaternion<double> q_r_init; // initial orientation for right_wp1 wrt fixed

	Eigen::Quaternion<double> q_l_init; // initial orientation for left_wp1 wrt fixed

	Eigen::Quaternion<double> q_fixed; // orientation of fixed frame wrt world

	int M, N; // Will be used to determine the # wps so that we can correctly 
			  // get the cart position as a fnc of s

	std::string parameter_file;

	CartVisualizationNode(ros::NodeHandle* n_input, const std::string & parameter_file_input);

	~CartVisualizationNode();

	void s_callback(const std_msgs::Float64ConstPtr & msg);

	void getParameters();

	void fillCartMarkerArray(visualization_msgs::MarkerArray & cart_msg);

	void getVizInformation(tf::Transform & tf_world_fixed, tf::Transform & tf_fixed_cart);


};


#endif