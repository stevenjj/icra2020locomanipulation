// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>

#include <avatar_locomanipulation/visualization_nodes/door_visualization_node.hpp>
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <iostream>
#include <fstream>

#include <math.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>

void visualize_door_and_waypoints(geometry_msgs::PoseArray & waypoints){

  // Initialize ROS node for publishing joint messages
  ros::NodeHandle n;
  ros::Rate loop_rate(20);
  DoorVisualizationNode door_obj(&n);

  // Waypoint Pose Publisher
	ros::Publisher hand_pose_pub = n.advertise<geometry_msgs::PoseArray>("waypoint_poses", 100);

  // Initialize Transforms and Messages
  ros::Publisher door_viz_pub = n.advertise<visualization_msgs::Marker>("door_visualization", 1000);
  visualization_msgs::Marker door_msg;

  // Transform Broadcasters for getting the wp1 in world then hinge in wp1
  // Initialize Transforms and Messages
  tf::TransformBroadcaster br_wp1;
  tf::TransformBroadcaster br_hinge;  
  // The transforms
  tf::Transform tf_world_hinge;
  // tf::Transform tf_wp1_hinge;
  door_obj.s_sub = door_obj.nh->subscribe<std_msgs::Float64>("/s_value", 1, boost::bind(&DoorVisualizationNode::s_callback, &door_obj, _1));
 
  while (ros::ok()){
    br_hinge.sendTransform(tf::StampedTransform(tf_world_hinge, ros::Time::now(), "world", "hinge_frame"));
    door_obj.getVizInformation(door_msg, tf_world_hinge);
    door_viz_pub.publish(door_msg);
    hand_pose_pub.publish(waypoints);

    ros::spinOnce();
    loop_rate.sleep();
  }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "door_visualization");
	
	// Initialize Param Handler
	ParamHandler param_handler;
	// Initialize the waypoint posearray
	geometry_msgs::PoseArray waypoint_poses;
	// Initialize the temp poses for building ^
	geometry_msgs::Pose temp;

	// Load the yaml file
	param_handler.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/door_open_trajectory_v3.yaml");

	int num_wps;
	param_handler.getInteger("num_waypoints", num_wps);
	std::string waypoint_string;
	double x, y, z, rx, ry, rz, rw;

	waypoint_poses.header.frame_id = "hinge_frame";

	for(int i=1; i<=num_wps; ++i){
		waypoint_string = "waypoint_" + std::to_string(i);

		param_handler.getNestedValue({waypoint_string, "x"}, x);
		param_handler.getNestedValue({waypoint_string, "y"}, y);
		param_handler.getNestedValue({waypoint_string, "z"}, z);
		param_handler.getNestedValue({waypoint_string, "rx"}, rx);
		param_handler.getNestedValue({waypoint_string, "ry"}, ry);
		param_handler.getNestedValue({waypoint_string, "rz"}, rz);
		param_handler.getNestedValue({waypoint_string, "rw"}, rw);

		temp.position.x = x; temp.position.y = y; temp.position.z = z;
		temp.orientation.x = rx; temp.orientation.y = ry;
		temp.orientation.z = rz; temp.orientation.w = rw;

		waypoint_poses.poses.push_back(temp);
	}

	visualize_door_and_waypoints(waypoint_poses);
}