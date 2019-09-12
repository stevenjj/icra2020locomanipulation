#include <Configuration.h>
#include <iostream>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <avatar_locomanipulation/models/robot_model.hpp>


// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"



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


  ParamHandler param_handler;
  param_handler.load_yaml_file(THIS_PACKAGE_PATH"stored_configurations/robot_door_final_configuration.yaml");  

  // Get the robot configuration vector
  std::vector<double> robot_q;
  param_handler.getVector("robot_configuration", robot_q);

  // Convert to Eigen data types
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_q.size());
  for(int i = 0; i < robot_q.size(); i++){
    q[i] = robot_q[i];
  }
  // Set outputs
  q_start = q;
  q_init = q_start;
}



void getInRightCOPFrame(Eigen::VectorXd & q_start){
  // Initialize ROS node for publishing joint messages
  ros::NodeHandle n;
  ros::Rate loop_rate(20);

  // Initialize Rviz translator
  ValRvizTranslator rviz_translator;

  tf::TransformListener listener;

  // Transform broadcaster
  tf::TransformBroadcaster      br_ik;
  tf::TransformBroadcaster      br_robot;
  // Joint State Publisher
  ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);

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

  valkyrie.q_current = q_start;
  Eigen::Quaternion<double> cur_ori;
  Eigen::Vector3d cur_pos;
  valkyrie.updateFullKinematics(q_start);
  valkyrie.updateGeometry(q_start);

  geometry_msgs::PoseStamped lfoot, rhand, pelvis;
  lfoot.header.frame_id = "world"; rhand.header.frame_id = "world"; pelvis.header.frame_id = "world";

  valkyrie.getFrameWorldPose("leftCOP_Frame", cur_pos, cur_ori);
  lfoot.pose.position.x = cur_pos[0]; lfoot.pose.position.y = cur_pos[1]; lfoot.pose.position.z = cur_pos[2];
  lfoot.pose.orientation.x = cur_ori.x();
  lfoot.pose.orientation.y = cur_ori.y();
  lfoot.pose.orientation.z = cur_ori.z();
  lfoot.pose.orientation.w = cur_ori.w();

  valkyrie.getFrameWorldPose("rightPalm", cur_pos, cur_ori);
  rhand.pose.position.x = cur_pos[0]; rhand.pose.position.y = cur_pos[1]; rhand.pose.position.z = cur_pos[2];
  rhand.pose.orientation.x = cur_ori.x();
  rhand.pose.orientation.y = cur_ori.y();
  rhand.pose.orientation.z = cur_ori.z();
  rhand.pose.orientation.w = cur_ori.w();

  valkyrie.getFrameWorldPose("pelvis", cur_pos, cur_ori);
  pelvis.pose.position.x = cur_pos[0]; pelvis.pose.position.y = cur_pos[1]; pelvis.pose.position.z = cur_pos[2];
  pelvis.pose.orientation.x = cur_ori.x();
  pelvis.pose.orientation.y = cur_ori.y();
  pelvis.pose.orientation.z = cur_ori.z();
  pelvis.pose.orientation.w = cur_ori.w();

  geometry_msgs::PoseStamped lfoot_rframe, rhand_rframe, pelvis_rframe;
  std::string to_frame;
  to_frame = "rightCOP_Frame";
  while (ros::ok()){
    tf::TransformListener::transformPose(to_frame, lfoot, lfoot_rframe);
    tf::TransformListener::transformPose(to_frame, rhand, rhand_rframe);
    tf::TransformListener::transformPose(to_frame, pelvis, pelvis_rframe);

    std::cout << "lfoot_rframe.pose.position.x: " << lfoot_rframe.pose.position.x << std::endl;
    std::cout << "lfoot_rframe.pose.position.y: " << lfoot_rframe.pose.position.y << std::endl;
    std::cout << "lfoot_rframe.pose.position.z: " << lfoot_rframe.pose.position.z << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
}


int main(int argc, char ** argv){
  ros::init(argc, argv, "test_ik_module");
	// X dimensional state vectors
  Eigen::VectorXd q_init;

  initialize_config(q_init);

  getInRightCOPFrame(q_init);
  
  return 0;
}