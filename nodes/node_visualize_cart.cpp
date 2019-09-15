// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>

#include <avatar_locomanipulation/visualization_nodes/cart_visualization_node.hpp>
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <iostream>
#include <fstream>

#include <math.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>

void visualize_cart_and_waypoints(geometry_msgs::PoseArray & waypoints){

	// Initialize ROS node for publishing joint messages
  ros::NodeHandle n;
  ros::Rate loop_rate(20);

  std::string parameter = "hand_trajectory/cart_parameters.yaml";
  CartVisualizationNode cart_obj(&n, parameter);

  // Waypoint Pose Publisher
	ros::Publisher hand_pose_pub = n.advertise<geometry_msgs::PoseArray>("waypoint_poses", 100);

  // Initialize Transforms and Messages
  ros::Publisher cart_viz_pub = n.advertise<visualization_msgs::MarkerArray>("cart_visualization", 1000);
  visualization_msgs::MarkerArray cart_msg;

  cart_obj.fillCartMarkerArray(cart_msg);

  // Transform Broadcasters for getting the wp1 in world then hinge in wp1
  // Initialize Transforms and Messages
  tf::TransformBroadcaster br_fixed;
  tf::TransformBroadcaster br_cart;  
  // The transforms
  tf::Transform tf_world_fixed;
  tf::Transform tf_fixed_cart;

  // tf::Transform tf_wp1_hinge;
  cart_obj.s_sub = cart_obj.nh->subscribe<std_msgs::Float64>("/s_value", 1, boost::bind(&CartVisualizationNode::s_callback, &cart_obj, _1));
 
  while (ros::ok()){
    
    cart_obj.getVizInformation(tf_world_fixed, tf_fixed_cart);

    br_fixed.sendTransform(tf::StampedTransform(tf_world_fixed, ros::Time::now(), "world", "fixed_frame"));
    br_cart.sendTransform(tf::StampedTransform(tf_fixed_cart, ros::Time::now(), "fixed_frame", "cart_frame"));

    cart_viz_pub.publish(cart_msg);
    hand_pose_pub.publish(waypoints);

    ros::spinOnce();
    loop_rate.sleep();
  }

}



int main(int argc, char **argv){
	ros::init(argc, argv, "cart_visualization");
	
	// Initialize Param Handler
	ParamHandler param_handler_left;
	ParamHandler param_handler_right;
	// Initialize the waypoint posearray
	geometry_msgs::PoseArray waypoint_poses;
	// Initialize the temp poses for building ^
	geometry_msgs::Pose temp;

	// Load the yaml file
	param_handler_left.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/cart_left_hand_trajectory.yaml");
	param_handler_right.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/cart_right_hand_trajectory.yaml");

	int num_wps;
	param_handler_left.getInteger("num_waypoints", num_wps);
	std::string left_waypoint_string, right_waypoint_string;
	double x, y, z, rx, ry, rz, rw;

	std::cout << "num_wps: " << num_wps << std::endl;

	waypoint_poses.header.frame_id = "fixed_frame";

	for(int i=1; i<=num_wps; ++i){
		left_waypoint_string = "waypoint_" + std::to_string(i);
		right_waypoint_string = "waypoint_" + std::to_string(i);

		param_handler_left.getNestedValue({left_waypoint_string, "x"}, x);
		param_handler_left.getNestedValue({left_waypoint_string, "y"}, y);
		param_handler_left.getNestedValue({left_waypoint_string, "z"}, z);
		param_handler_left.getNestedValue({left_waypoint_string, "rx"}, rx);
		param_handler_left.getNestedValue({left_waypoint_string, "ry"}, ry);
		param_handler_left.getNestedValue({left_waypoint_string, "rz"}, rz);
		param_handler_left.getNestedValue({left_waypoint_string, "rw"}, rw);

		temp.position.x = x; temp.position.y = y; temp.position.z = z;
		temp.orientation.x = rx; temp.orientation.y = ry;
		temp.orientation.z = rz; temp.orientation.w = rw;

		waypoint_poses.poses.push_back(temp);

		param_handler_right.getNestedValue({right_waypoint_string, "x"}, x);
		param_handler_right.getNestedValue({right_waypoint_string, "y"}, y);
		param_handler_right.getNestedValue({right_waypoint_string, "z"}, z);
		param_handler_right.getNestedValue({right_waypoint_string, "rx"}, rx);
		param_handler_right.getNestedValue({right_waypoint_string, "ry"}, ry);
		param_handler_right.getNestedValue({right_waypoint_string, "rz"}, rz);
		param_handler_right.getNestedValue({right_waypoint_string, "rw"}, rw);

		temp.position.x = x; temp.position.y = y; temp.position.z = z;
		temp.orientation.x = rx; temp.orientation.y = ry;
		temp.orientation.z = rz; temp.orientation.w = rw;

		waypoint_poses.poses.push_back(temp);
	}

	visualize_cart_and_waypoints(waypoint_poses);
}