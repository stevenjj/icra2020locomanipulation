#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

#include <avatar_locomanipulation/helpers/NeuralNetModel.hpp>

# include <cstdlib>
# include <iostream>
# include <iomanip>
# include <cmath>

#include <math.h>
#include <time.h>
#include <random>

#include <avatar_locomanipulation/helpers/IOUtilities.hpp>
#include <Eigen/Dense>
#include <avatar_locomanipulation/BinaryClassifierQuery.h>

#include <avatar_locomanipulation/models/robot_model.hpp>

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

#define CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE 0
#define CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE 1

#define CONTACT_TRANSITION_DATA_LEFT_HAND 0
#define CONTACT_TRANSITION_DATA_RIGHT_HAND 1
#define CONTACT_TRANSITION_DATA_BOTH_HANDS 2


struct PositionScore{
	double f_score;
	Eigen::Vector3d position;
};

void fill_poses(geometry_msgs::PoseArray & poses);

void fill_hand_position_markers(visualization_msgs::MarkerArray & hand_markers, std::vector<PositionScore> & ps);

Eigen::Vector3d quatToVec(const Eigen::Quaterniond & ori){
	Eigen::AngleAxisd tmp_ori(ori.normalized()); // gets the normalized version of ori and sets it to an angle axis representation
  Eigen::Vector3d ori_vec = tmp_ori.axis()*tmp_ori.angle();
  return ori_vec;
}

void normalizeInputCalculate(const Eigen::VectorXd & x_in, const Eigen::VectorXd & data_mean, const Eigen::VectorXd & data_std_dev, Eigen::VectorXd & x_normalized) {
  // std::cout << x_in.rows() << " " << data_mean.rows() << " " << data_std_dev.rows() << std::endl;
  x_normalized = (x_in - data_mean).cwiseQuotient(data_std_dev);
}

int main(int argc, char ** argv){
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

	nn_landing_foot_pos[0] = 0.0; nn_landing_foot_pos[1] = 0.3; nn_landing_foot_pos[2] = 0.0; 
	
	nn_pelvis_pos[0] = 0.0; nn_pelvis_pos[1] = 0.15; nn_pelvis_pos[2] = 0.9;

	// Create the blank feature vectors 
  Eigen::VectorXd rawDatum(32);
  Eigen::VectorXd datum(32);
	Eigen::MatrixXd data(1053,32);
	// Blank vector for prediction f_scores
	Eigen::MatrixXd pred(100,1);
	std::vector<Eigen::Vector3d> rhand_position;

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

  // For rand number generation (For Orientation)
  unsigned int seed_number = 1;
	std::mt19937 generator;
	generator.seed(seed_number);
	std::uniform_real_distribution<double> u_distribution_double(-M_PI, M_PI);
	Eigen::VectorXd randx(100), randy(100), randz(100);
	// // Generate our list of random numbers
	// for(int k=0; k<100; ++k){
	// 	randx[k] = u_distribution_double(generator);
	// 	randy[k] = u_distribution_double(generator);
	// 	randz[k] = u_distribution_double(generator);
	// }

	// Fill the rawDatum
		// Only the nn_right_hand_start_pos and tmp_ori_vec3 will change
	rawDatum << nn_stance_origin, nn_manipulation_type, 
		nn_swing_foot_start_pos, quatToVec(nn_swing_foot_start_ori),
		nn_pelvis_pos, quatToVec(nn_pelvis_ori),
		nn_landing_foot_pos, quatToVec(nn_landing_foot_ori),
		nn_right_hand_start_pos, tmp_ori_vec3,
		nn_left_hand_start_pos, quatToVec(nn_left_hand_start_ori);


	int i=0;
	for(int q=-20; q<=20; q+=5){
		// Set the x position
		nn_right_hand_start_pos[0] = static_cast<double>(q) / 100.0;

		for(int r=-80; r<=-20; r+=5){
			// Set the y position
			nn_right_hand_start_pos[1] = static_cast<double>(r) / 100.0;

			for(int s=80; s<=120; s+=5){
				// Set the z position
				nn_right_hand_start_pos[2] = static_cast<double>(s) / 100.0;
				// Reset the Prediction Sum
				prediction_sum = 0.0;

				// // Loop thru all of the random orientations
				// for(int j=0; j<100; ++j){

					// Set the rhand ori and normalize
					// nn_right_hand_start_ori[0] = randx[j]; nn_right_hand_start_ori[1] = randy[j]; nn_right_hand_start_ori[2] = randz[j]; 
					// tmp_ori_vec3 = nn_right_hand_start_ori.normalized();
					Eigen::Quaterniond rhand_door_start_ori;
					rhand_door_start_ori.x() = 0.0;
					rhand_door_start_ori.y() = -0.707;
					rhand_door_start_ori.z() = 0.0;
					rhand_door_start_ori.w() = 0.707;

					rhand_position.push_back(nn_right_hand_start_pos);

					tmp_ori_vec3 = quatToVec(rhand_door_start_ori);

					// Set the current right hand position and orientation in the rawDatum
					rawDatum[20] = nn_right_hand_start_pos[0]; rawDatum[21] = nn_right_hand_start_pos[1]; rawDatum[22] = nn_right_hand_start_pos[2];
					rawDatum[23] = tmp_ori_vec3[0]; rawDatum[24] = tmp_ori_vec3[1]; rawDatum[25] = tmp_ori_vec3[2];

					normalizeInputCalculate(rawDatum, mean, std_dev, datum);

					data.row(i) = datum;
					++i;

				// } // random orientation loop closed 

				// // Once we have 100 rows of data send to the model and get prediction
				// pred = nn_transition.GetOutput(data);
				// // Set the position for our struct
				// temp.position = nn_right_hand_start_pos;
				// // Get the sum or predictions for the 100 orientations
				// for(int w=0; w<100; ++w){
				// 	prediction_sum += pred(w,0);
				// }
				// // f_score is the avg over 100 orientations
				// temp.f_score = prediction_sum/100.0;
				// Add temp to the vector of structs
				// original.push_back(temp);

			}// z for loop closed
		}// y for loop closed
	}// x for loop closed

	std::cout << "before\n";
	pred = nn_transition.GetOutput(data);
	std::cout << "after\n";
	// Set the position for our struct
	for(int p=0; p<rhand_position.size(); ++p){
		temp.position = rhand_position[p];
		temp.f_score = pred(p,0);
		original.push_back(temp);
	}
	

	std::cout << "Generated all of the data and f scores\n";

	// NOW WE HAVE ALL OF THE DATA GENERATED AND NEED TO STANDARDIZE IT

	double max_val = original[0].f_score;
	// Find the maximum f_score
	for(int i=1; i < original.size(); ++i){
		if(original[i].f_score >= max_val) max_val = original[i].f_score;
	}
	// Standardize it
	for(int i=0; i < original.size(); ++i){
		original[i].f_score = original[i].f_score/max_val;
	}


	// NOW IT HAS BEEN STANDARDIZED AND NEEDS TO BE VIZ'd

	std::cout << "Standardized all of the data\n";

	std::cout << "original.size(): " << original.size() << std::endl;

	// Initialize ROS node for publishing joint messages
	ros::init(argc, argv, "test_rviz_feasibility_viz");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);
	// Define MarkerArray (Hand Positions)
	visualization_msgs::MarkerArray hand_markers;
	hand_markers.markers[original.size()];
	// Define PoseArray (Pose of pelvis, rfoot, lfoot)
	geometry_msgs::PoseArray poses;
	// Waypoint Publisher
	ros::Publisher hand_markers_pub = n.advertise<visualization_msgs::MarkerArray>("hand_positions", 100);
	// Hand Pose Publisher
	ros::Publisher poses_pub = n.advertise<geometry_msgs::PoseArray>("body_poses", 100);
	
	fill_poses(poses);
	fill_hand_position_markers(hand_markers, original);


	while (ros::ok()){
	    
		hand_markers_pub.publish(hand_markers);
		poses_pub.publish(poses);
		std::cout << "publishing\n";

		ros::spinOnce();
		loop_rate.sleep();		
	}


  return 0;
}



void fill_hand_position_markers(visualization_msgs::MarkerArray & hand_markers, std::vector<PositionScore> & ps){
	// Define the marker for filling up array
	visualization_msgs::Marker temp;
	double f_score;

	// Build a marker for each of the points in space 
	for(int w=0; w<=ps.size(); ++w){
		f_score = ps[w].f_score;

		std::cout << "tic\n";

		// Fill the Marker 
		temp.pose.position.x = ps[w].position[0];
		temp.pose.position.y = ps[w].position[1];
		temp.pose.position.z = ps[w].position[2];	
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
		// temp.text = score_string;
		temp.color.r = (1.0-f_score);
		temp.color.b = 0.0f;
		temp.color.g = (f_score);
		temp.color.a = 0.3;
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
	pose.position.y = 0.15;
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
	pose.position.y = 0.3;
	pose.position.z = 0.0;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;
	poses.poses.push_back(pose);
}