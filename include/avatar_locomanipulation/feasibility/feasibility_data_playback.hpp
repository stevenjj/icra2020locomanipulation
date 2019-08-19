#ifndef ALM_FEASIBILITY_DATA_PLAYBACK_H
#define ALM_FEASIBILITY_DATA_PLAYBACK_H

#include <Configuration.h>

// Robot Model and Configuration Trajectory Generator
#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>

// Special data types
#include <avatar_locomanipulation/data_types/footstep.hpp>
#include <avatar_locomanipulation/data_types/trajectory_SE3.hpp>

// Parameter Loader and Saver
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>
#include <fstream>

#include <iostream>

// Helpers
#include <avatar_locomanipulation/helpers/orientation_utils.hpp>

// This class loads a positive transition data and recomputes the trajectory.
// it also plays back a symmetric version of the data.
class FeasibilityDataPlayBack{
public:
	FeasibilityDataPlayBack();
	~FeasibilityDataPlayBack();

	// Initialize the playback routine
	void common_initialization();
    void setRobotModel(std::shared_ptr<RobotModel> & robot_model_in);

    // Load the walking pattern generator file path
	void loadParamFile(const std::string filepath);
	// Load the stored positive transition data
	void loadData(const std::string filepath);

	// compute the configuration trajectory to ensure correctness of the data
	// also compute the symmetric version of this data
	bool playback();
	void produceSymmetricData();

    // Parameter Handler
    ParamHandler param_handler;

	std::shared_ptr<RobotModel> robot_model;
	std::shared_ptr<ConfigTrajectoryGenerator> ctg;

	// Data Walking Parameters
	double walking_com_height = 0.95;
	double walking_double_support_time = 0.45; // AKA transfer time
	double walking_single_support_time = 1.0; // AKA swing time	
	double walking_settling_percentage = 0.999;
	double walking_swing_height = 0.1;

	std::string manipulation_type = ""; // "left_hand" "right_hand" "both_hands" 
	std::string stance_foot = ""; // "left_foot" / "right_foot"

	int N_resolution = 60; // Resolution discretization to use for solving the transition trajectory
	int loaded_seed_number = 1;


	// robot starting configuration
	Eigen::VectorXd q_start;	

	Eigen::Vector3d swing_foot_position;
	Eigen::Quaterniond swing_foot_orientation;

	Footstep landing_footstep;
	Eigen::Vector3d landing_foot_position;
	Eigen::Quaterniond landing_foot_orientation;

	Eigen::Vector3d right_hand_position;
	Eigen::Quaterniond right_hand_orientation;

	Eigen::Vector3d left_hand_position;
	Eigen::Quaterniond left_hand_orientation;

	// data gen case
	int data_gen_manipulation_case;
	int data_gen_stance_case;

private:
	void getParamVec(const std::string param_name, Eigen::VectorXd & vec);
	void getParamPos(const std::string param_name, Eigen::Vector3d & pos);
	void getParamOri(const std::string param_name, Eigen::Quaterniond & ori);


};



#endif