#ifndef ALM_BAG_VISUALIZATION_H
#define ALM_BAG_VISUALIZATION_H

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

class BagVisualizationNode{
public:
	ros::NodeHandle* nh;

	ros::Subscriber s_sub;

	double s_current; // curent s for determining bag position

	double radius; // turn radius

	double rotation_angle; // angle b/w pickup and putdown 

	Eigen::Vector3d center_frame_pos;

	double lift_height; // height of handle (assumed 1/2 of door size)

	double pose_spacing; // spacing b/w each wp

	Eigen::Quaternion<double> q_init; // initial orientation for wp1

	Eigen::Quaternion<double> q_fixed; // orientation of fixed center frame wrt world

	int M, N; // Will be used to determine the # wps so that we can correctly 
			  // get the bag position as a fnc of s

	std::string parameter_file;

	BagVisualizationNode(ros::NodeHandle* n_input, const std::string & parameter_file_input);

	~BagVisualizationNode();

	void s_callback(const std_msgs::Float64ConstPtr & msg);

	void getParameters();

	void fillBagMarkerArray(visualization_msgs::MarkerArray & bag_msg);

	void getVizInformation(tf::Transform & tf_world_center, tf::Transform & tf_center_bag);


};


#endif