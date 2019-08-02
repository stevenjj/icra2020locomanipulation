// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <iostream>
#include <fstream>

#include <math.h>


int main(int argc, char **argv){

	// Initialize ROS node for publishing joint messages
	ros::init(argc, argv, "test_rviz_waypoints");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);

	// Initialize Rviz translator
	ValRvizTranslator rviz_translator;

	// Initializa Param Handler
	ParamHandler param_handler;

	// Transform broadcaster
	tf::TransformBroadcaster      br_ik;
	tf::TransformBroadcaster      br_robot;	
	tf::TransformBroadcaster 	  br_hinge;
	tf::TransformBroadcaster 	  br_test;

	// Joint State Publisher
	ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
	ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);
	// Door Publisher
	ros::Publisher door_pub = n.advertise<visualization_msgs::Marker>("door", 100);
	// Handle Publisher
	ros::Publisher handle_pub = n.advertise<visualization_msgs::Marker>("handle", 100);
	// Hinge Publisher
	ros::Publisher hinge_pub = n.advertise<visualization_msgs::Marker>("hinge", 100);
	// Waypoint Publisher
	ros::Publisher waypoint_pub = n.advertise<visualization_msgs::MarkerArray>("waypoints", 100);
	// Hand Pose Publisher
	ros::Publisher hand_pose_pub = n.advertise<geometry_msgs::PoseArray>("hand_poses", 100);


	// Initialize Transforms and Messages
	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_end;
	tf::Transform tf_world_hinge;
	tf::Transform tf_hinge_rpalm;
	tf::TransformListener listener;

	sensor_msgs::JointState joint_msg_init; 
	sensor_msgs::JointState joint_msg_end; 
	visualization_msgs::Marker door_msg, handle_msg, hinge_msg, temp;
	visualization_msgs::MarkerArray waypoints;
	geometry_msgs::PoseArray hand_poses;
	geometry_msgs::Pose pose;

	std::cout << "Initialize Valkyrie Model" << std::endl;
	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
	std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
	std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

 	RobotModel valkyrie(filename, meshDir, srdf_filename);
	Eigen::VectorXd  q_start(valkyrie.getDimQ()); q_start.setZero();
	Eigen::VectorXd  q_end(valkyrie.getDimQ()); q_end.setZero();

	// Initialize Linear positions
	q_start[2] = 1.13177;
	q_end[0] = -1.0;
	q_end[2] = q_start[2];	
	// Initialize Quaternion components to Identity
	q_start[6] = 1.0;
	q_end[6] = 1.0;

	// Set q_end:
	q_end[valkyrie.getJointIndex("leftHipPitch")] = -0.3; 
	q_end[valkyrie.getJointIndex("rightHipPitch")] = -0.3;  
	q_end[valkyrie.getJointIndex("leftKneePitch")] = 0.6;  
	q_end[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
	q_end[valkyrie.getJointIndex("leftAnklePitch")] = -0.3; 
	q_end[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;

	q_end[valkyrie.getJointIndex("rightShoulderPitch")] = -0.2; 
	q_end[valkyrie.getJointIndex("rightShoulderRoll")] = 1.1;  
	q_end[valkyrie.getJointIndex("rightElbowPitch")] = 0.4;  
	q_end[valkyrie.getJointIndex("rightForearmYaw")] = 1.5; 	
	q_end[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2; 
	q_end[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;  
	q_end[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
	q_end[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;  
 

	// Visualize q_start and q_end in RVIZ
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_start, tf_world_pelvis_init, joint_msg_init);
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_end, tf_world_pelvis_end, joint_msg_end);

	// Add the Hinge frame
	tf_world_hinge.setOrigin(tf::Vector3(1.5, -0.5, 1.125));
	tf_world_hinge.setRotation(tf::Quaternion(0, 0, 0.707, 0.707));
	
	// Fill the Handle marker msg
	handle_msg.pose.position.x = 1.475;
	handle_msg.pose.position.y = 0.4;
	handle_msg.pose.position.z = 1.125;
	handle_msg.pose.orientation.y = 0.;
	handle_msg.pose.orientation.z = 0.;
	handle_msg.pose.orientation.w = 1.;
	handle_msg.scale.x = 0.05;
	handle_msg.scale.y = 0.1;
	handle_msg.scale.z = 0.03;
	handle_msg.header.frame_id = "world";
	handle_msg.text = "box";
	// handle_msg.header.stamp = ros::Time::now();
	handle_msg.color.r = 1.0f;
	handle_msg.color.g = 0.0f;
	handle_msg.color.b = 0.0f;
	handle_msg.color.a = 1.0;
	// handle_msg.lifetime = ros::Duration();
	handle_msg.action = visualization_msgs::Marker::ADD;
	handle_msg.type = visualization_msgs::Marker::CUBE;

	// Fill the Hinge marker msg
	hinge_msg.pose.position.x = 1.5;
	hinge_msg.pose.position.y = -0.5;
	hinge_msg.pose.position.z = 1.125;
	hinge_msg.pose.orientation.y = 0.;
	hinge_msg.pose.orientation.z = 0.;
	hinge_msg.pose.orientation.w = 1.;
	hinge_msg.scale.x = 0.05;
	hinge_msg.scale.y = 0.03;
	hinge_msg.scale.z = 0.1;
	hinge_msg.header.frame_id = "world";
	hinge_msg.text = "box";
	// hinge_msg.header.stamp = ros::Time::now();
	hinge_msg.color.r = 1.0f;
	hinge_msg.color.g = 0.0f;
	hinge_msg.color.b = 0.0f;
	hinge_msg.color.a = 1.0;
	// hinge_msg.lifetime = ros::Duration();
	hinge_msg.action = visualization_msgs::Marker::ADD;
	hinge_msg.type = visualization_msgs::Marker::CUBE;

	// Fill the Door marker msg
	door_msg.pose.position.x = 1.5;
	door_msg.pose.position.y = 0.;
	door_msg.pose.position.z = 1.125;
	door_msg.pose.orientation.y = 0.;
	door_msg.pose.orientation.z = 0.;
	door_msg.pose.orientation.w = 1.;
	door_msg.scale.x = 0.05;
	door_msg.scale.y = 1.0;
	door_msg.scale.z = 2.25;
	door_msg.header.frame_id = "world";
	door_msg.text = "box";
	// door_msg.header.stamp = ros::Time::now();
	door_msg.color.r = 0.0f;
	door_msg.color.g = 1.0f;
	door_msg.color.b = 0.0f;
	door_msg.color.a = 1.0;
	// door_msg.lifetime = ros::Duration();
	door_msg.action = visualization_msgs::Marker::ADD;
	door_msg.type = visualization_msgs::Marker::CUBE;


	// Load the yaml file
	param_handler.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/door_trajectory.yaml");

	char point[12];
	std::vector<double> xs, ys, zs, rxs, rys, rzs, rws;
	double x, y, z, rx, ry, rz, rw;
	double radius = 0.9;

	// Grab the waypoint and pose data from the yaml file
	for(int i=1; i<11; ++i){
		snprintf(point, sizeof(char)*32, "waypoint_%d", i);
		param_handler.getNestedValue({point, "x"}, x);
		param_handler.getNestedValue({point, "y"}, y);
		param_handler.getNestedValue({point, "z"}, z);

		xs.push_back(x);
		ys.push_back(y);
		zs.push_back(z);

		param_handler.getNestedValue({point, "rx"}, rx);
		param_handler.getNestedValue({point, "ry"}, ry);
		param_handler.getNestedValue({point, "rz"}, rz);
		param_handler.getNestedValue({point, "rw"}, rw);

		rxs.push_back(rx);
		rys.push_back(ry);
		rzs.push_back(rz);
		rws.push_back(rw);
		
	}

	hand_poses.header.frame_id = "hinge_frame";
	
	for(int j=0; j<xs.size(); ++j){
		// Fill the MarkerArray waypoints
    	temp.pose.position.x = (xs[j])*radius;
		temp.pose.position.y = (ys[j])*radius + 0.075;
		temp.pose.position.z = zs[j];	
		temp.pose.orientation.x = 0.;	
		temp.pose.orientation.y = 0.;
		temp.pose.orientation.z = 0.;
		temp.pose.orientation.w = 1.;
		temp.ns = "waypoints";
		temp.id = j;
		temp.scale.x = 0.02;
		temp.scale.y = 0.02;
		temp.scale.z = 0.02;
		temp.header.frame_id = "hinge_frame";
		temp.text = point;
		temp.color.r = 0.0f;
		temp.color.g = 0.0f;
		temp.color.b = 1.0f;
		temp.color.a = 1.0;
		temp.action = visualization_msgs::Marker::ADD;
		temp.type = visualization_msgs::Marker::CUBE;
		waypoints.markers.push_back(temp);

		// Fill the PoseArray hand_poses
		pose.position.x = (xs[j])*radius;
		pose.position.y = (ys[j])*radius + 0.075;
		pose.position.z = zs[j];
		pose.orientation.x = rxs[j];
		pose.orientation.y = rys[j];
		pose.orientation.z = rzs[j];
		pose.orientation.w = rws[j];
		hand_poses.poses.push_back(pose);
    }

    // Add the Palm frame
	tf_hinge_rpalm.setOrigin(tf::Vector3(0.9, 0.075, 0.0));
	tf_hinge_rpalm.setRotation(tf::Quaternion(0.0, -0.707, 0.0, 0.707));


	while (ros::ok()){
	    br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
	    robot_joint_state_pub.publish(joint_msg_init);

	    br_hinge.sendTransform(tf::StampedTransform(tf_world_hinge, ros::Time::now(), "world", "hinge_frame"));
	    br_test.sendTransform(tf::StampedTransform(tf_hinge_rpalm, ros::Time::now(), "hinge_frame", "rpalm_frame"));
	    
	    door_pub.publish(door_msg);
	    handle_pub.publish(handle_msg);
	    hinge_pub.publish(hinge_msg);
	    waypoint_pub.publish(waypoints);
	    hand_pose_pub.publish(hand_poses);

	    br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
	    robot_ik_joint_state_pub.publish(joint_msg_end);
		ros::spinOnce();
		loop_rate.sleep();		
	}
	return 0;

}