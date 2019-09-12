#include <ros/ros.h>
#include <avatar_locomanipulation/BinaryClassifierQuery.h>

#include <avatar_locomanipulation/planners/a_star_planner.hpp>
#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/data_types/manipulation_function.hpp>
#include <avatar_locomanipulation/data_types/footstep.hpp>

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <math.h>
#include <time.h>
#include <random>

#define CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE 0
#define CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE 1

#define CONTACT_TRANSITION_DATA_LEFT_HAND 0
#define CONTACT_TRANSITION_DATA_RIGHT_HAND 1
#define CONTACT_TRANSITION_DATA_BOTH_HANDS 2


int main(int argc, char ** argv){
	// Initialize ros
	ros::init(argc, argv, "test_visualize_feasibility_data");
	// Seed number for rand number for generating 
	unsigned int seed_number = 1;
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
	Eigen::Vector3d nn_right_hand_start_ori;
	Eigen::Vector3d nn_left_hand_start_pos;
	Eigen::Quaterniond nn_left_hand_start_ori;
	int nn_stance_origin;
	int nn_manipulation_type;
	// Singular prediction result, which will be added to the sum before generating the avg over 100 points per position
	double prediction_result, prediction_sum, prediction_avg;
	// X Vector
	std::vector<double> x;
	// For addToXVector
	Eigen::AngleAxisd tmp_aa;
	Eigen::Vector3d tmp_ori_vec3;
	// For rand number generation
	std::mt19937 generator;
	generator.seed(seed_number);
	std::uniform_real_distribution<double> u_distribution_double(-M_PI, M_PI);
	std::vector<double> randx, randy, randz;
	// Generate our list of random numbers
	for(int k=0; k<100; ++k){
		randx.push_back(u_distribution_double(generator));
		randy.push_back(u_distribution_double(generator));
		randz.push_back(u_distribution_double(generator));
	}
	// For the Yaml emitter	
	YAML::Emitter out;
	out << YAML::BeginMap;
	// Base string for yaml outputs
	std::string score_string;
	std::string position_string;
	
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

	// Set all of the values we want fixed
	nn_swing_foot_start_pos[0] = 0.0; nn_swing_foot_start_pos[1] = -0.2; nn_swing_foot_start_pos[2] = 0.0; 

	nn_landing_foot_pos[0] = 0.0; nn_landing_foot_pos[1] = -0.2; nn_landing_foot_pos[2] = 0.0; 
	
	nn_pelvis_pos[0] = 0.0; nn_pelvis_pos[1] = -0.1; nn_pelvis_pos[2] = 0.9;

	int w = 1;

	for(int q=-50; q<=50; q+=5){
		// Set the x position
		nn_right_hand_start_pos[0] = static_cast<double>(q) / 100.0;

		for(int r=-30; r<=80; r+=5){
			// Set the y position
			nn_right_hand_start_pos[1] = static_cast<double>(r) / 100.0;

			for(int s=0; s<=150; s+=5){
				// Set the z position
				nn_right_hand_start_pos[2] = static_cast<double>(s) / 100.0;
				
				// Reset the Prediction Sum
				prediction_sum = 0.0;
				// Loop thru all of the random orientations
				for(int j=0; j<100; ++j){
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
						// Right hand Position
					for(int i = 0; i < nn_right_hand_start_pos.size(); i++){
						x.push_back(nn_right_hand_start_pos[i]);
					}
						// Right hand Orientation
					nn_right_hand_start_ori[0] = randx[j]; nn_right_hand_start_ori[1] = randy[j]; nn_right_hand_start_ori[2] = randz[j]; 
					tmp_ori_vec3 = nn_right_hand_start_ori.normalized();
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
						prediction_sum += prediction_result;
					}else{
						ROS_ERROR("Failed to call service locomanipulation_feasibility_classifier");
					}
				}// For loop randomizing 100 right hand ori's
				prediction_avg = prediction_sum/100.;
				score_string = "feasibility_score_" + std::to_string(w);
				position_string = "right_hand_position_" + std::to_string(w);
				std::cout << "position_string: " << position_string << std::endl;
				std::cout << "score_string: " << score_string << std::endl;
				++w;
				data_saver::emit_position(out, position_string, nn_right_hand_start_pos);
				data_saver::emit_value(out, score_string, prediction_avg);
				
			}// For loop going through rhand z position
		}// For loop going through rhand y position
	}// For loop going through rhand x position

	out << YAML::EndMap;

	std::ofstream file_output_stream("right_hand_feasibility.yaml");
	file_output_stream << out.c_str();
}



