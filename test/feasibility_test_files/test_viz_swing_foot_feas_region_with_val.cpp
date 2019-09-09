// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <avatar_locomanipulation/feasibility/feasibility.hpp>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include <time.h>
#include <random>

#include <avatar_locomanipulation/helpers/NeuralNetModel.hpp>

#include <avatar_locomanipulation/helpers/IOUtilities.hpp>
#include <Eigen/Dense>
#include <avatar_locomanipulation/BinaryClassifierQuery.h>
#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

#define CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE 0
#define CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE 1

#define CONTACT_TRANSITION_DATA_LEFT_HAND 0
#define CONTACT_TRANSITION_DATA_RIGHT_HAND 1
#define CONTACT_TRANSITION_DATA_BOTH_HANDS 2

void visualize_robot_and_markers(Eigen::VectorXd & q_start, visualization_msgs::MarkerArray & foot_markers){
  // Initialize ROS node for publishing joint messages
  ros::NodeHandle n;
  ros::Rate loop_rate(20);

  // Waypoint Publisher
	ros::Publisher foot_markers_pub = n.advertise<visualization_msgs::MarkerArray>("foot_positions", 100);

  // Initialize Rviz translator
  ValRvizTranslator rviz_translator;

  // Transform broadcaster
  tf::TransformBroadcaster      br_ik;
  tf::TransformBroadcaster      br_robot;
  // Joint State Publisher
  ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);

  // Initialize Transforms and Messages
  tf::Transform tf_world_pelvis_init;
  tf::Transform tf_world_pelvis_end;

  sensor_msgs::JointState joint_msg_init;
  sensor_msgs::JointState joint_msg_end;

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

		foot_markers_pub.publish(foot_markers);
		std::cout << "publishing\n";

		ros::spinOnce();
		loop_rate.sleep();
  }
}

struct PositionScore{
	double f_score;
	Eigen::Vector3d position;
};


void fill_hand_position_markers(visualization_msgs::MarkerArray & foot_markers, std::vector<PositionScore> & ps);

Eigen::Vector3d quatToVec(const Eigen::Quaterniond & ori){
	Eigen::AngleAxisd tmp_ori(ori.normalized()); // gets the normalized version of ori and sets it to an angle axis representation
  Eigen::Vector3d ori_vec = tmp_ori.axis()*tmp_ori.angle();
  return ori_vec;
}

void normalizeInputCalculate(const Eigen::VectorXd & x_in, const Eigen::VectorXd & data_mean, const Eigen::VectorXd & data_std_dev, Eigen::VectorXd & x_normalized) {
  // std::cout << x_in.rows() << " " << data_mean.rows() << " " << data_std_dev.rows() << std::endl;
  x_normalized = (x_in - data_mean).cwiseQuotient(data_std_dev);
}


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
  q_start[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;

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

int main(int argc, char **argv){

	// Find and load the model
  ParamHandler param_handler;
  std::string model_path = "/home/ryan/nasa_ws/src/avatar_locomanipulation/src/python_src/rh_transitions/learned_model/model.yaml";
  
  std::cout << "Loading Model..." << std::endl;
  myYAML::Node model = myYAML::LoadFile(model_path);
  NeuralNetModel nn_transition(model, false);
  std::cout << "Loaded" << std::endl;

  // Create the structures which will hold the position and score
  std::vector<PositionScore> original;
  PositionScore temp;
  //Define feature vector members
	Eigen::Vector3d nn_swing_foot_start_pos;
	Eigen::Quaterniond nn_swing_foot_start_ori;
	Eigen::Vector3d nn_pelvis_pos;
	Eigen::Quaterniond nn_pelvis_ori;
	Eigen::Vector3d nn_landing_foot_pos;
	Eigen::Quaterniond nn_landing_foot_ori;
	Eigen::Vector3d nn_right_hand_start_pos;
	Eigen::Vector3d nn_right_hand_start_ori;
	Eigen::Vector3d nn_left_hand_start_pos;
	Eigen::Quaterniond nn_left_hand_start_ori;
	double nn_stance_origin;
	double nn_manipulation_type;
	// For addToXVector
	Eigen::AngleAxisd tmp_aa;
	Eigen::Vector3d tmp_ori_vec3;
	// For getting the results 
	double prediction_sum, prediction_avg;
	prediction_sum = 0;

	// Initialize all of our values
	nn_stance_origin = CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE; 
	nn_manipulation_type = CONTACT_TRANSITION_DATA_RIGHT_HAND; 

	nn_swing_foot_start_pos.setZero();
	nn_swing_foot_start_ori.setIdentity();

	nn_pelvis_pos.setZero();
	nn_pelvis_ori.setIdentity();

	nn_landing_foot_pos.setZero();
	nn_landing_foot_ori.setIdentity();

	nn_right_hand_start_pos.setZero();
	nn_right_hand_start_ori.setZero();

	nn_left_hand_start_pos.setZero();
	nn_left_hand_start_ori.setIdentity();

	tmp_ori_vec3.setZero();

	// Set all of the NZ/Non Identity values we want fixed
	nn_swing_foot_start_pos[0] = 0.0; nn_swing_foot_start_pos[1] = 0.3; nn_swing_foot_start_pos[2] = 0.0; 

	nn_right_hand_start_pos[0] = 0.1335; nn_right_hand_start_pos[1] = -0.3471; nn_right_hand_start_pos[2] = 0.9566;
	  
	nn_landing_foot_pos[2] = 0.0;

	nn_pelvis_pos[0] = 0.0; nn_pelvis_pos[1] = 0.15; nn_pelvis_pos[2] = 0.9;

	Eigen::Quaterniond rhand_door_start_ori;
	rhand_door_start_ori.x() = 0.0;
	rhand_door_start_ori.y() = -0.707;
	rhand_door_start_ori.z() = 0.0;
	rhand_door_start_ori.w() = 0.707;
	tmp_ori_vec3 = quatToVec(rhand_door_start_ori);

	// Create the blank feature vectors 
  Eigen::VectorXd rawDatum(32);
  Eigen::VectorXd datum(32);
	Eigen::MatrixXd data(357,32);
	// Blank vector for prediction f_scores
	Eigen::MatrixXd pred(357,1);
	std::vector<Eigen::Vector3d> rfoot_position;

	 //Normalization Params
  param_handler.load_yaml_file("/home/ryan/nasa_ws/src/avatar_locomanipulation/nn_models/baseline_11500pts/lh_rflh_lfrh_lfrh_rfbh_rfbh_lf/normalization_params.yaml");
  std::vector<double> vmean;
  param_handler.getVector("x_train_mean", vmean);
  std::vector<double> vstd_dev;
  param_handler.getVector("x_train_std", vstd_dev);

  Eigen::VectorXd mean(32);
  Eigen::VectorXd std_dev(32);
  for (int ii = 0; ii < 32; ii++){
    mean[ii] = vmean[ii];
    std_dev[ii] = vstd_dev[ii];
  }

  rawDatum << nn_stance_origin, nn_manipulation_type, 
		nn_swing_foot_start_pos, quatToVec(nn_swing_foot_start_ori),
		nn_pelvis_pos, quatToVec(nn_pelvis_ori),
		nn_landing_foot_pos, quatToVec(nn_landing_foot_ori),
		nn_right_hand_start_pos, tmp_ori_vec3,
		nn_left_hand_start_pos, quatToVec(nn_left_hand_start_ori);

	int i=0;
	for(int q=-40; q<=40; q+=5){
		// Set the x position
		nn_landing_foot_pos[0] = static_cast<double>(q) / 100.0;

		for(int r=-20; r<=80; r+=5){
			// Set the y position
			nn_landing_foot_pos[1] = static_cast<double>(r) / 100.0;

			rfoot_position.push_back(nn_landing_foot_pos);

			// Set the current right hand position and orientation in the rawDatum
			rawDatum[14] = nn_landing_foot_pos[0]; rawDatum[15] = nn_landing_foot_pos[1]; rawDatum[16] = nn_landing_foot_pos[2];

			normalizeInputCalculate(rawDatum, mean, std_dev, datum);

			data.row(i) = datum;
			++i;
		} // close y pos loop
	}// close x pos loop


	std::cout << "before\n";
	pred = nn_transition.GetOutput(data);
	std::cout << "after\n";
	// Set the position for our struct
	for(int p=0; p<rfoot_position.size(); ++p){
		temp.position = rfoot_position[p];
		temp.f_score = pred(p,0);
		original.push_back(temp);
	}

  ros::init(argc, argv, "generate_rfoot_pretty_pictures");

  Eigen::VectorXd q_init;
  initialize_config(q_init);

  // Define MarkerArray (Hand Positions)
	visualization_msgs::MarkerArray foot_markers;
	foot_markers.markers[original.size()];

	fill_hand_position_markers(foot_markers, original);

	visualize_robot_and_markers(q_init, foot_markers);

  return 0;

}


void fill_hand_position_markers(visualization_msgs::MarkerArray & foot_markers, std::vector<PositionScore> & ps){
	// Define the marker for filling up array
	visualization_msgs::Marker temp;
	double f_score;

	// Build a marker for each of the points in space 
	for(int w=0; w<=ps.size(); ++w){
		f_score = ps[w].f_score;


		// Fill the Marker 
		temp.pose.position.x = ps[w].position[0] + 0.0353245;
		temp.pose.position.y = ps[w].position[1] - 0.132544;
		temp.pose.position.z = ps[w].position[2] - 0.130611;	
		temp.pose.orientation.x = 0.;	
		temp.pose.orientation.y = 0.;
		temp.pose.orientation.z = 0.;
		temp.pose.orientation.w = 1.;
		temp.ns = "foot_positions";
		temp.id = w;
		temp.scale.x = 0.05;
		temp.scale.y = 0.05;
		temp.scale.z = 0.02;
		temp.header.frame_id = "world";
		// temp.text = score_string;
		// if((ps[w].position[1] <= 0.19) || (ps[w].position[1] >= 0.41) || (ps[w].position[0] <= -0.21) || (ps[w].position[0] >= 0.21)){
		// 	temp.color.r = 0.5;
		// 	temp.color.b = 0.5;
		// 	temp.color.g = 0.5;
		// }else{
			temp.color.r = (1.0-f_score);
			temp.color.b = 0.0f;
			temp.color.g = (f_score);
		// }
		
		temp.color.a = 0.3;
		temp.action = visualization_msgs::Marker::ADD;
		temp.type = visualization_msgs::Marker::CUBE;
		foot_markers.markers.push_back(temp);
	}
}
