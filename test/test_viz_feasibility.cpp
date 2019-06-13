// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/valkyrie_model.hpp>

#include <avatar_locomanipulation/feasibility/feasibility.hpp>

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

	// Initialize Transforms and Messages
	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_end;

	sensor_msgs::JointState joint_msg_init;
	sensor_msgs::JointState joint_msg_end;

	// Initialize Robot Model
	ValkyrieModel valkyrie;
	Eigen::VectorXd  q_start(valkyrie.getDimQ()); q_start.setZero();
	Eigen::VectorXd  q_end(valkyrie.getDimQ()); q_end.setZero();

	// Initialize feasibility object
	feasibility	fiz;
	fiz.InverseKinematicsTop(40);

	q_start = fiz.q_start;
	q_end = fiz.q_end;

	// Visualize q_start and q_end in RVIZ
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_start, tf_world_pelvis_init, joint_msg_init);
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_end, tf_world_pelvis_end, joint_msg_end);

	while (ros::ok()){
	    br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
	    robot_joint_state_pub.publish(joint_msg_init);

	    br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
	    robot_ik_joint_state_pub.publish(joint_msg_end);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;

}
