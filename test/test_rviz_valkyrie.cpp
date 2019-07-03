// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>


int main(int argc, char **argv){
	// Initialize ROS node for publishing joint messages
	ros::init(argc, argv, "test_rviz");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);

	// Initialize Rviz translator
	ValRvizTranslator rviz_translator;

	// Transform broadcaster
	tf::TransformBroadcaster      br_ik;
	tf::TransformBroadcaster      br_robot;	
	// Joint State Publisher
	ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
	ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);
	// Box Publisher
	ros::Publisher box_pub = n.advertise<visualization_msgs::Marker>("box", 100);
	
	// Initialize Transforms and Messages
	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_end;
	tf::TransformListener listener;

	sensor_msgs::JointState joint_msg_init; 
	sensor_msgs::JointState joint_msg_end; 
	visualization_msgs::Marker box_msg;

	// Initialize Robot Model
	ValkyrieModel valkyrie;
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

	
	// Fill the box marker msg
	box_msg.pose.position.x = 0.;
	box_msg.pose.position.y = 0.;
	box_msg.pose.position.z = 1.13177;
	box_msg.pose.orientation.y = 0.;
	box_msg.pose.orientation.z = 0.;
	box_msg.pose.orientation.w = 1.;
	box_msg.scale.x = 1.;
	box_msg.scale.y = 1.;
	box_msg.scale.z = 1.;
	box_msg.header.frame_id = "world";
	box_msg.text = "box";
	box_msg.header.stamp = ros::Time::now();
	box_msg.color.r = 0.0f;
	box_msg.color.g = 1.0f;
	box_msg.color.b = 0.0f;
	box_msg.color.a = 1.0;
	box_msg.lifetime = ros::Duration();
	box_msg.action = visualization_msgs::Marker::ADD;
	box_msg.type = visualization_msgs::Marker::CUBE;

	while (ros::ok()){
	    br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
	    robot_joint_state_pub.publish(joint_msg_init);
	    
	    box_pub.publish(box_msg);

	    br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
	    robot_ik_joint_state_pub.publish(joint_msg_end);
		ros::spinOnce();
		loop_rate.sleep();		
	}
	return 0;

}
