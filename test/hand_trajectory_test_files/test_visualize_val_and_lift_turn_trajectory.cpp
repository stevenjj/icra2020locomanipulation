// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>

#include <avatar_locomanipulation/models/robot_model.hpp>

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <iostream>
#include <fstream>

#include <math.h>


void initialize_config(Eigen::VectorXd & q_init){

  std::cout << "Initialize Valkyrie Model" << std::endl;
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  RobotModel valkyrie(urdf_filename, meshDir_, srdf_filename);

  // X dimensional state vectors
  Eigen::VectorXd q_start;

  // Set origin at 0.0
  q_start = Eigen::VectorXd::Zero(valkyrie.getDimQ());
  double theta = 0.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q

  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[valkyrie.getJointIndex("leftHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightAnklePitch")] = 0.0;//-0.3;

  q_start[valkyrie.getJointIndex("rightShoulderPitch")] = 0.2;
  q_start[valkyrie.getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie.getJointIndex("rightElbowPitch")] = 1.0 ; //0.4;
  q_start[valkyrie.getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}


void visualize_robot_and_waypoints(Eigen::VectorXd & q_start, geometry_msgs::PoseArray & waypoints){
  // Initialize ROS node for publishing joint messages
  ros::NodeHandle n;
  ros::Rate loop_rate(20);

  // Initialize Rviz translator
  ValRvizTranslator rviz_translator;

  // Transform broadcaster
  tf::TransformBroadcaster      br_robot;
  // Joint State Publisher
  ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);
  // Waypoint Pose Publisher
	ros::Publisher hand_pose_pub = n.advertise<geometry_msgs::PoseArray>("waypoint_poses", 100);

  // Initialize Transforms and Messages
  tf::Transform tf_world_pelvis_init;

  sensor_msgs::JointState joint_msg_init;

  // Initialize Robot Model
  std::cout << "Initialize Valkyrie Model" << std::endl;
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  RobotModel valkyrie(urdf_filename, meshDir_, srdf_filename);

  // Visualize q_start and q_end in RVIZ
  rviz_translator.populate_joint_state_msg(valkyrie.model, q_start, tf_world_pelvis_init, joint_msg_init);

  while (ros::ok()){
      br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
      robot_joint_state_pub.publish(joint_msg_init);

      hand_pose_pub.publish(waypoints);

    ros::spinOnce();
    loop_rate.sleep();
  }
}


int main(int argc, char **argv){

	ros::init(argc, argv, "test_custom_trajectory");

	// Initialize Param Handler
	ParamHandler param_handler;
	// Initialize the waypoint posearray
	geometry_msgs::PoseArray waypoint_poses;
	// Initialize the temp poses for building ^
	geometry_msgs::Pose temp;
	// Initialize config vector
	Eigen::VectorXd q_start;

	initialize_config(q_start);

	// Load the yaml file
	param_handler.load_yaml_file(THIS_PACKAGE_PATH"custom_hand_lift_turn_trajectory.yaml");

	int num_wps;
	param_handler.getInteger("num_waypoints", num_wps);
	std::string waypoint_string;
	double x, y, z, rx, ry, rz, rw;

	waypoint_poses.header.frame_id = "world";

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

	visualize_robot_and_waypoints(q_start, waypoint_poses);
}