#ifndef ALM_CONFIG_TRAJECTORY_GENERATOR_H
#define ALM_CONFIG_TRAJECTORY_GENERATOR_H

#include <Configuration.h>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>
#include <avatar_locomanipulation/data_types/trajectory_SE3.hpp>

// This class outputs a trajectory of robot configuration q from
//     - a starting configuration, q, and a sequence of footsteps
// 	   - a desired hand/s pose/s trajectory, starting config, and a sequence of footsteps


// To initialize this object, first call these functions:
// setRobotModel() 
// initializeDiscretization()

class ConfigTrajectoryGenerator{
public:
	// Constructors
	ConfigTrajectoryGenerator(); 
	ConfigTrajectoryGenerator(std::shared_ptr<RobotModel> & robot_model_in); // Accepts a robot model to use
	// Destructors
	~ConfigTrajectoryGenerator();

	// Public Member Functions
    void setRobotModel(std::shared_ptr<RobotModel> & robot_model_in);
    void initializeDiscretization(const int & N_size_in);

    // Computes an initial configuration that ensures that the robot's feet are flat on the ground.
    // If the feet are already flat on the ground q_out is set to the input q_guess.   
    void computeInitialConfigForFlatGround(const Eigen::VectorXd & q_guess, Eigen::VectorXd & q_out); 

    // Given an initial configuration, 
    void computeConfigurationTrajectory(const Eigen::VectorXd & q_init, const std::vector<Footstep> & input_footstep_list);


    int getDiscretizationSize();

    // Sets the SE3 trajectories for the left and right hands
    void setLeftHandTrajectory(const TrajSE3 & traj_SE3_left_hand_in);
    void setRightHandTrajectory(const TrajSE3 & traj_SE3_left_hand_in);    

    // Public Member Variables
	std::shared_ptr<RobotModel> robot_model;
    IKModule starting_config_ik_module; // IK module to use for computing the starting configuration
    IKModule ik_module; // 

	WalkingPatternGenerator wpg;

	// Trajectory containers
	TrajEuclidean   traj_q_config; 	// Trajectory of configurations q
	TrajSE3         traj_SE3_left_hand; // Left Hand Trajectories (if it exists)
	TrajSE3         traj_SE3_right_hand; // Right Hand Trajectories (if it exists)

	int N_size = 100;


private:
	bool use_left_hand = false;
	bool use_right_hand = false;



};


#endif