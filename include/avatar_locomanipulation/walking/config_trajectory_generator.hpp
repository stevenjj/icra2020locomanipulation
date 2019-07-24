#ifndef ALM_CONFIG_TRAJECTORY_GENERATOR_H
#define ALM_CONFIG_TRAJECTORY_GENERATOR_H

#include <Configuration.h>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>
#include <avatar_locomanipulation/data_types/trajectory_SE3.hpp>

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>


// This class outputs a trajectory of robot configuration q from
//     - a starting configuration, q, and a sequence of footsteps
// 	   - a desired hand/s pose/s trajectory, starting config, and a sequence of footsteps


// To initialize this object, these functions must be called:
// setRobotModel() 
// initializeDiscretization()
// initializeTasks()
// reinitializeTaskStack()
// setStartingConfig()

class ConfigTrajectoryGenerator{
public:
	// Constructors
	ConfigTrajectoryGenerator(); 
	ConfigTrajectoryGenerator(std::shared_ptr<RobotModel> & robot_model_in); // Accepts a robot model to use
	// Destructors
	~ConfigTrajectoryGenerator();

	// Required Initializations
    void setRobotModel(std::shared_ptr<RobotModel> & robot_model_in);
    void initializeDiscretization(const int & N_size_in);
    void initializeTasks();
	void reinitializeTaskStack();

	void setStartingConfig(const Eigen::VectorXd & q_start_in);

	// Setters to active certain task types. When these functions have been called,
	// reinitializeTaskStack() has to be called again.
	void setUseRightHand(bool use_right_hand_in);
	void setUseLeftHand(bool use_left_hand_in);
	void setUseTorsoJointPosition(bool use_torso_joint_position_in);

    // Computes an initial configuration that ensures that the robot's feet are flat on the ground.
    // If the feet are already flat on the ground q_out is set to the input q_guess.   
    void computeInitialConfigForFlatGround(const Eigen::VectorXd & q_guess, Eigen::VectorXd & q_out); 

    // Given an initial configuration, 
    void computeConfigurationTrajectory(const Eigen::VectorXd & q_init, const std::vector<Footstep> & input_footstep_list);

    // returns N_size
    int getDiscretizationSize();

    // Sets the SE3 trajectories for the left and right hands
    void setLeftHandTrajectory(const TrajSE3 & traj_SE3_left_hand_in);
    void setRightHandTrajectory(const TrajSE3 & traj_SE3_left_hand_in);    

    // Public Member Variables
	std::shared_ptr<RobotModel> robot_model;
    IKModule starting_config_ik_module; // IK module to use for computing the starting configuration
    IKModule ik_module; 
	WalkingPatternGenerator wpg;


	// Set Tasks
	std::shared_ptr<Task> pelvis_ori_task;
	std::shared_ptr<Task> com_task;

	std::shared_ptr<Task> rfoot_task;
	std::shared_ptr<Task> lfoot_task;

	std::shared_ptr<Task> rhand_task;
	std::shared_ptr<Task> lhand_task;

	std::shared_ptr<Task> torso_posture_task;
	std::shared_ptr<Task> neck_posture_task;
	std::shared_ptr<Task> rarm_posture_task;
	std::shared_ptr<Task> larm_posture_task;

	std::shared_ptr<Task> task_stack;

	// Trajectory containers
	TrajEuclidean   traj_q_config; 	// Trajectory of configurations q
	TrajSE3         traj_SE3_left_hand; // Left Hand Trajectories (if it exists)
	TrajSE3         traj_SE3_right_hand; // Right Hand Trajectories (if it exists)

	Eigen::VectorXd q_start;

	int N_size = 100;


private:
	void createTaskStack();

	// Reference will use the initial joint configuration in traj_q_config.
	void getSelectedPostureTaskReferences(std::vector<std::string> & selected_names, Eigen::VectorXd & q_ref);

	bool use_right_hand = true;
	bool use_left_hand = false;
	bool use_torso_joint_position = false;

};


#endif