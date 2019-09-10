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
#define CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE 0
#define CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE 1

#define CONTACT_TRANSITION_DATA_LEFT_HAND 0
#define CONTACT_TRANSITION_DATA_RIGHT_HAND 1
#define CONTACT_TRANSITION_DATA_BOTH_HANDS 2

struct PositionScore{
	double f_score;
	Eigen::Vector3d position;
};

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
void fill_hand_position_markers(visualization_msgs::MarkerArray & foot_markers, std::vector<PositionScore> & ps);


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

int main(int argc, char ** argv){

	// Initialize ros
	ros::init(argc, argv, "generate_rfoot_pretty_pictures");
	//Service name for neural network
	std::string service_name = "locomanipulation_feasibility_classifier";
	std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
	// Define client to neural network
	ros::ServiceClient classifier_client;   
	classifier_client = ros_node->serviceClient<avatar_locomanipulation::BinaryClassifierQuery>(service_name);
	// Prepare classifier service request
	avatar_locomanipulation::BinaryClassifierQuery classifier_srv;
	int classifier_input_dim = 32;
	//Define feature vector members
	Eigen::Vector3d nn_swing_foot_start_pos;
	Eigen::Quaterniond nn_swing_foot_start_ori;
	Eigen::Vector3d nn_pelvis_pos;
	Eigen::Quaterniond nn_pelvis_ori;
	Eigen::Vector3d nn_landing_foot_pos;
	Eigen::Quaterniond nn_landing_foot_ori;
	Eigen::Vector3d nn_right_hand_start_pos;
	Eigen::Quaterniond nn_right_hand_start_ori;
	Eigen::Vector3d nn_left_hand_start_pos;
	Eigen::Quaterniond nn_left_hand_start_ori;
	int nn_stance_origin;
	int nn_manipulation_type;
	// Singular prediction result, which will be added to the sum before generating the avg over 100 points per position
	double prediction_result;
	// Create the structures which will hold the position and score
	std::vector<PositionScore> original;
	PositionScore temp;

	// X Vector
	std::vector<double> x;
	// For addToXVector
	Eigen::AngleAxisd tmp_aa;
	Eigen::Vector3d tmp_ori_vec3;

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
	nn_right_hand_start_ori.setIdentity();

	nn_left_hand_start_pos.setZero();
	nn_left_hand_start_ori.setIdentity();

	tmp_ori_vec3.setZero();

	// Set all of the NZ/Non Identity values we want fixed
	nn_swing_foot_start_pos[0] = 0.0; nn_swing_foot_start_pos[1] = 0.3; nn_swing_foot_start_pos[2] = 0.0; 

	nn_right_hand_start_pos[0] = 0.1335; nn_right_hand_start_pos[1] = -0.3471; nn_right_hand_start_pos[2] = 0.9566;
	  
	nn_landing_foot_pos[2] = 0.0;

	nn_pelvis_pos[0] = 0.0; nn_pelvis_pos[1] = 0.15; nn_pelvis_pos[2] = 0.9;

	double xmin, xmax, ymin, ymax;
	double dx = 0.05;
	int N, M;
	N = 16;
	M = 20;
	xmin = -0.4; xmax = 0.4; ymin = -0.2; ymax = 0.8;

	for(int i=0; i<N+1; ++i){
		// x landing position
		nn_landing_foot_pos[0] = xmin + (static_cast<double>(i))*dx;///static_cast<double>(N)

		for(int j=0; j<M+1; ++j){
			// y landing position 
			nn_landing_foot_pos[1] = ymin + (static_cast<double>(j))*dx;///static_cast<double>(M)

			std::cout << "nn_landing_foot_pos[0] = " <<  nn_landing_foot_pos[0] << std::endl;
			std::cout << "nn_landing_foot_pos[1] = " <<  nn_landing_foot_pos[1] << std::endl;


			// Prepare x vector
			x.clear();
			// Prepare classifier input
			classifier_srv.request.x.clear();
			classifier_srv.request.x.reserve(classifier_input_dim);

			// Populate the X Vector
			x.push_back(nn_stance_origin);
			x.push_back(nn_manipulation_type);

			// Swing Foot Start
			tmp_aa = nn_swing_foot_start_ori.normalized(); // gets the normalized version of ori and sets it to an angle axis representation
			tmp_ori_vec3 = tmp_aa.axis()*tmp_aa.angle();

			for(int i = 0; i < nn_swing_foot_start_pos.size(); i++){
				x.push_back(nn_swing_foot_start_pos[i]);
			}
			for(int i = 0; i < tmp_ori_vec3.size(); i++){
				x.push_back(tmp_ori_vec3[i]);
			}

			// Pelvis
			tmp_aa = nn_pelvis_ori.normalized(); // gets the normalized version of ori and sets it to an angle axis representation
			tmp_ori_vec3 = tmp_aa.axis()*tmp_aa.angle();

			for(int i = 0; i < nn_pelvis_pos.size(); i++){
				x.push_back(nn_pelvis_pos[i]);
			}
			for(int i = 0; i < tmp_ori_vec3.size(); i++){
				x.push_back(tmp_ori_vec3[i]);
			}

			// Swing Foot Land
			tmp_aa = nn_landing_foot_ori.normalized(); // gets the normalized version of ori and sets it to an angle axis representation
			tmp_ori_vec3 = tmp_aa.axis()*tmp_aa.angle();

			for(int i = 0; i < nn_landing_foot_pos.size(); i++){
				x.push_back(nn_landing_foot_pos[i]);
			}
			for(int i = 0; i < tmp_ori_vec3.size(); i++){
				x.push_back(tmp_ori_vec3[i]);
			}

			// Right hand
			tmp_aa = nn_right_hand_start_ori.normalized(); // gets the normalized version of ori and sets it to an angle axis representation
			tmp_ori_vec3 = tmp_aa.axis()*tmp_aa.angle();

			for(int i = 0; i < nn_right_hand_start_pos.size(); i++){
				x.push_back(nn_right_hand_start_pos[i]);
			}
			for(int i = 0; i < tmp_ori_vec3.size(); i++){
				x.push_back(tmp_ori_vec3[i]);
			}

			// Left Hand
			tmp_aa = nn_left_hand_start_ori.normalized(); // gets the normalized version of ori and sets it to an angle axis representation
			tmp_ori_vec3 = tmp_aa.axis()*tmp_aa.angle();

			for(int i = 0; i < nn_left_hand_start_pos.size(); i++){
				x.push_back(nn_left_hand_start_pos[i]);
			}
			for(int i = 0; i < tmp_ori_vec3.size(); i++){
				x.push_back(tmp_ori_vec3[i]);
			}	
			// Reset prediction result to a negative value
			prediction_result = -1.0;

			classifier_srv.request.x = x;
			// Call the classifier client
			if (classifier_client.call(classifier_srv)){
				prediction_result = classifier_srv.response.y;
				std::cout << "classifier_srv.response.y: " << classifier_srv.response.y << std::endl;
			}else{
				ROS_ERROR("Failed to call service locomanipulation_feasibility_classifier");
			}
			temp.f_score = prediction_result;
			temp.position = nn_landing_foot_pos;
			original.push_back(temp);
		}// end y for loop
	}// end x for loop

	Eigen::VectorXd q_init;
	initialize_config(q_init);

	// Define MarkerArray (Hand Positions)
	visualization_msgs::MarkerArray foot_markers;
	foot_markers.markers[original.size()];
	std::cout << "s1\n";

	fill_hand_position_markers(foot_markers, original);
	std::cout << "s2\n";

	visualize_robot_and_markers(q_init, foot_markers);

	return 0;
}


void fill_hand_position_markers(visualization_msgs::MarkerArray & foot_markers, std::vector<PositionScore> & ps){
	// Define the marker for filling up array
	visualization_msgs::Marker temp;
	double f_score;
	std::cout << "ps.size(): " << ps.size() << std::endl;

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
		if(f_score > 0.5){
			temp.color.g = 1.0f;
			temp.color.b = 0.0f;
			temp.color.r = 0.0f;
		}
		// else{
			temp.color.r = (1.0-f_score);
			temp.color.b = 0.0f;
			temp.color.g = (f_score);
		// }
			
		// }
		
		temp.color.a = 0.3;
		temp.action = visualization_msgs::Marker::ADD;
		temp.type = visualization_msgs::Marker::CUBE;
		foot_markers.markers.push_back(temp);
	}
}
