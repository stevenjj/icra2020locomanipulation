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
	void loadParamfile(const std::string filepath);
	// Load the stored positive transition data
	void loadData(const std::string filepath);

	// compute the configuration trajectory to ensure correctness of the data
	// also compute the symmetric version of this data
	void playback();
	void produceSymmetricData();

	// Data Parameters
	double max_reach = 0.4; // maximum forward step
	double min_reach = -0.2; // maximum backward step
	double max_width = 0.4; // maximum side step
	double min_width = 0.2; // minimum side step
	double max_theta = 0.6; // maximum foot angle w.r.t stance
	double min_theta = -0.15; // minimum foot angle w.r.t stance

	double convex_hull_percentage = 0.9; // Percentage of convex hull to use for randomization of pelvis location
	double pelvis_height_min = 0.9;
	double pelvis_height_max = 1.05;

	double com_height_min = 0.9;
	double com_height_max = 1.0;

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

	// data gen case
	int data_gen_manipulation_case;
	int data_gen_stance_case;

	// Trajectory configuration
	TrajEuclidean  traj_q_config;


};



#endif