#include <avatar_locomanipulation/helpers/param_handler.hpp>
// ROS and Viz types
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

void fill_poses(geometry_msgs::PoseArray & poses);

void fill_hand_position_markers(visualization_msgs::MarkerArray & hand_markers);


int main(int argc, char ** argv){
	// Initialize ROS node for publishing joint messages
	ros::init(argc, argv, "test_rviz_feasibility_viz");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);
	// Define MarkerArray (Hand Positions)
	visualization_msgs::MarkerArray hand_markers;
	// Define PoseArray (Pose of pelvis, rfoot, lfoot)
	geometry_msgs::PoseArray poses;
	// Waypoint Publisher
	ros::Publisher hand_markers_pub = n.advertise<visualization_msgs::MarkerArray>("hand_positions", 100);
	// Hand Pose Publisher
	ros::Publisher poses_pub = n.advertise<geometry_msgs::PoseArray>("body_poses", 100);
	
	fill_poses(poses);
	fill_hand_position_markers(hand_markers);


	while (ros::ok()){
	    
		hand_markers_pub.publish(hand_markers);
		poses_pub.publish(poses);
		std::cout << "publishing\n";

		ros::spinOnce();
		loop_rate.sleep();		
	}
	return 0;
}


void fill_hand_position_markers(visualization_msgs::MarkerArray & hand_markers){
	// Define the marker for filling up array
	visualization_msgs::Marker temp;
	// Define strings for getting from yaml
	std::string position_string, score_string;
	double x, y, z, f_score;
	// Example loading of the file
	ParamHandler param_handler;
	param_handler.load_yaml_file("right_hand_standardized_feasibility.yaml");

	// Build a marker for each of the points in space //9261
	for(int w=1; w<=14973; ++w){
		score_string = "feasibility_score_" + std::to_string(w);
		position_string = "right_hand_position_" + std::to_string(w);
		// Get the feasibility score
		param_handler.getValue(score_string, f_score);
		// Get the rhand position
		param_handler.getNestedValue({position_string, "x"}, x);
		param_handler.getNestedValue({position_string, "y"}, y);
		param_handler.getNestedValue({position_string, "z"}, z);
		// Fill the Marker 
		temp.pose.position.x = x;
		temp.pose.position.y = y;
		temp.pose.position.z = z;	
		temp.pose.orientation.x = 0.;	
		temp.pose.orientation.y = 0.;
		temp.pose.orientation.z = 0.;
		temp.pose.orientation.w = 1.;
		temp.ns = "hand_positions";
		temp.id = w;
		temp.scale.x = 0.02;
		temp.scale.y = 0.02;
		temp.scale.z = 0.02;
		temp.header.frame_id = "map";
		temp.text = score_string;
		temp.color.r = (1.0-f_score);
		temp.color.g = 0.0f;
		temp.color.b = (f_score);
		temp.color.a = 0.7;
		temp.action = visualization_msgs::Marker::ADD;
		temp.type = visualization_msgs::Marker::CUBE;
		hand_markers.markers.push_back(temp);
	}
}


void fill_poses(geometry_msgs::PoseArray & poses){
	// Define the pose 
	geometry_msgs::Pose pose;

	poses.header.frame_id = "map";
	// First add pelvis to poses
	pose.position.x = 0.0;
	pose.position.y = -0.1;
	pose.position.z = 0.9;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;
	poses.poses.push_back(pose);
	// Next add the right foot
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;
	poses.poses.push_back(pose);
	// Last add the left foot
	pose.position.x = 0.0;
	pose.position.y = -0.2;
	pose.position.z = 0.0;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;
	poses.poses.push_back(pose);
}