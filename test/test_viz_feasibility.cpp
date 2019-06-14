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
	Eigen::VectorXd  q_data(valkyrie.getDimQ()); q_data.setZero();

	// Initialize feasibility object
	feasibility	fiz;
	fiz.InverseKinematicsTop(300);

	// Eigen::Vector3d p1(1,1,0);
	// Eigen::Vector3d p2(1,0,0);
	// Eigen::Vector3d p3(0,1,0);
	// Eigen::Vector3d p4(0,0,0);
	// Eigen::Vector3d p5(2.5,2.5,0);
	//
	// math_utils::Point p1_p; p1_p.x = p1[0]; p1_p.y = p1[1];
	// math_utils::Point p2_p; p2_p.x = p2[0]; p2_p.y = p2[1];
	// math_utils::Point p3_p; p3_p.x = p3[0]; p3_p.y = p3[1];
	// math_utils::Point p4_p; p4_p.x = p4[0]; p4_p.y = p4[1];
	// math_utils::Point p5_p; p5_p.x = p5[0]; p5_p.y = p5[1];
	//
	// std::vector<math_utils::Point> vec_points;
	// vec_points.push_back(p1_p);
	// vec_points.push_back(p2_p);
	// vec_points.push_back(p3_p);
	// vec_points.push_back(p4_p);
	// vec_points.push_back(p5_p);
	// for(int i = 0; i < vec_points.size(); i++){
	// 	std::cout << "hull vec_points: " << vec_points[i].x << ", " << vec_points[i].y << std::endl;
	// }
	//
	// std::vector<math_utils::Point> hull_vertices =  math_utils::convexHull(vec_points);
	// for(int i = 0; i < hull_vertices.size(); i++){
	// 	std::cout << "hull vertex: " << hull_vertices[i].x << ", " << hull_vertices[i].y << std::endl;
	// }

	q_start = fiz.q_start;
	q_end = fiz.q_end;
	q_data = fiz.q_data;

	// Visualize q_start and q_end in RVIZ
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_start, tf_world_pelvis_init, joint_msg_init);
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_data, tf_world_pelvis_end, joint_msg_end);

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
