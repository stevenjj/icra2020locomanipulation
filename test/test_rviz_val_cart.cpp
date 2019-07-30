// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>


int main(int argc, char **argv){
	// Initialize ROS node for publishing joint messages
	ros::init(argc, argv, "test_rviz_cart");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);

	// Initialize Rviz translator
	ValRvizTranslator rviz_translator;

	// Transform broadcaster
	tf::TransformBroadcaster      br_ik;
	tf::TransformBroadcaster      br_robot;	
	tf::TransformBroadcaster	  br_cart;
	// Joint State Publisher
	ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
	ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);
	ros::Publisher cart_joint_state_pub = n.advertise<sensor_msgs::JointState>("cart/joint_states", 10);
	
	// Initialize Transforms and Messages
	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_end;
	tf::Transform tf_world_base_link;
	tf::TransformListener listener;

	sensor_msgs::JointState joint_msg_init; 
	sensor_msgs::JointState joint_msg_end;
	sensor_msgs::JointState cart_msg; 
	// visualization_msgs::Marker box_msg;

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

	filename = THIS_PACKAGE_PATH"models/test_cart.urdf";
	meshDir  = THIS_PACKAGE_PATH"models/cart/";

	// Initialize Cart RobotModel
	RobotModel cart(filename, meshDir);
	// Define the configuration of the cart
	Eigen::VectorXd cart_config(cart.getDimQ()); cart_config.setZero();


	cart_config[0] = -0.05;  cart_config[1] = -0.0065;  cart_config[2] = -0.03;
	// cart_config[0] = -0.06;  cart_config[1] = -0.0085;  cart_config[2] = -0.04;
	double theta1 = 0;//M_PI/4.0;	
	Eigen::AngleAxis<double> bb(theta1, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left	
	Eigen::Quaternion<double> quat_init; quat_init =  bb;
	cart_config[3] = quat_init.x();// 0.0;	
	cart_config[4] = quat_init.y(); //0.0;
	cart_config[5] = quat_init.z(); //sin(theta/2.0);
	cart_config[6] = quat_init.w(); //cos(theta/2.0);
 

	// Visualize q_start and q_end in RVIZ
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_start, tf_world_pelvis_init, joint_msg_init);
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_end, tf_world_pelvis_end, joint_msg_end);
	rviz_translator.populate_joint_state_msg(cart.model, cart_config, tf_world_base_link, cart_msg);

	while (ros::ok()){
	    br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
	    robot_joint_state_pub.publish(joint_msg_init);

	    br_cart.sendTransform(tf::StampedTransform(tf_world_base_link, ros::Time::now(), "world",  "base_link"));
	    cart_joint_state_pub.publish(cart_msg);

	    br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
	    robot_ik_joint_state_pub.publish(joint_msg_end);
		ros::spinOnce();
		loop_rate.sleep();		
	}
	return 0;

}
