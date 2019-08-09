#ifndef ALM_FEASIBILITY_DATA_GENERATOR_H
#define ALM_FEASIBILITY_DATA_GENERATOR_H

#include <math.h>
#include <time.h>

#include <Configuration.h>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>
#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>
#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>

#include <avatar_locomanipulation/data_types/footstep.hpp>
#include <avatar_locomanipulation/data_types/trajectory_SE3.hpp>

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>

#include <avatar_locomanipulation/helpers/orientation_utils.hpp>


// Parameter Loader and Saver
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

class FeasibilityDataGenerator{
public:
	FeasibilityDataGenerator();
	~FeasibilityDataGenerator();

    void setRobotModel(std::shared_ptr<RobotModel> & robot_model_in);

    // Parameter Handler
    ParamHandler param_handler;

    // Public Member Variables
	std::shared_ptr<RobotModel> robot_model;
	std::shared_ptr<IKModule> ik_start_config_module;
	std::shared_ptr<ConfigTrajectoryGenerator> ctg;

	std::shared_ptr<Task> left_foot_task;
	std::shared_ptr<Task> right_foot_task;
	std::shared_ptr<Task> pelvis_ori;
	std::shared_ptr<Task> com_task;

	void initialize_modules();

	// data generation parameters
	double max_reach = 0.4;
	double min_reach = -0.2;
	double max_width = 0.4;
	double min_width = 0.2;
	double max_theta = 0.6;
	double min_theta = -0.15;

	double com_height_min = 0.9;
	double com_height_max = 1.0;

	double walking_com_height = 0.95;
	double walking_double_support_time = 0.45; // AKA transfer time
	double walking_single_support_time = 1.0; // AKA swing time	
	double walking_swing_height = 0.1;

	// robot configuration data types
	Eigen::VectorXd q_min;
	Eigen::VectorXd q_max;	
	Eigen::VectorXd q_start;	

	// Trajectory configuration
	TrajEuclidean   traj_q_config;

};

#endif