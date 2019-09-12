#ifndef ALM_CONFIG_TRAJECTORY_GENERATOR_H
#define ALM_CONFIG_TRAJECTORY_GENERATOR_H

#include <Configuration.h>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>
#include <avatar_locomanipulation/data_types/trajectory_SE3.hpp>

#include <avatar_locomanipulation/data_types/manipulation_function.hpp>

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>

#include <avatar_locomanipulation/helpers/orientation_utils.hpp>

#define CONFIG_TRAJECTORY_ROBOT_LEFT_SIDE 0
#define CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE 1

#define CONFIG_TRAJECTORY_VERBOSITY_LEVEL_0 0 // No text output
#define CONFIG_TRAJECTORY_VERBOSITY_LEVEL_1 1 // Only outputs the result of the complete trajectory
#define CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2 2 // Outputs the lower levels plus outputs the IK convergence result and error norm of each step of the trajectory
#define CONFIG_TRAJECTORY_VERBOSITY_LEVEL_3 3 // Outputs the lower levels plus outputs the IK iterations of each step of the trajectory
#define CONFIG_TRAJECTORY_VERBOSITY_LEVEL_4 4 // Outputs the lower levels plus outputs the final IK result summary of each step of the trajectory

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
	ConfigTrajectoryGenerator(std::shared_ptr<RobotModel> & robot_model_in, const int & N_size_in); // robot model to use and the discretization factor

	// Destructors
	~ConfigTrajectoryGenerator();

	// Required Initializations
	void commonInitialization();

    void setRobotModel(std::shared_ptr<RobotModel> & robot_model_in);
    void initializeDiscretization(const int & N_size_in);
    void initializeTasks();
	void reinitializeTaskStack();

	// Set the starting configuration
	void setStartingConfig(const Eigen::VectorXd & q_start_in);

	// Sets the current configuration
	void setCurrentConfig(const Eigen::VectorXd & q_current_in);

	// Setters to active certain task types. When these functions have been called,
	// reinitializeTaskStack() has to be called again.
	void setUseRightHand(bool use_right_hand_in);
	void setUseLeftHand(bool use_left_hand_in);
	void setUseTorsoJointPosition(bool use_torso_joint_position_in);

	// Sets the verbosity level from 0 to 4.
	void setVerbosityLevel(int verbosity_level_in);

    // Computes an initial configuration that ensures that the robot's feet are flat on the ground.
    // and the CoM is at a height equal to the z dimension of the DCM's virtual repellant point
    // If the feet are already flat on the ground q_out is set to the input q_guess.   
    bool computeInitialConfigForFlatGround(const Eigen::VectorXd & q_guess, Eigen::VectorXd & q_out); 

    // Given an initial configuration and footstep data list input, compute the task space walking trajectory.
    // Warning: If hand tasks are enabled, they need to have been set already.
    bool computeConfigurationTrajectory(const Eigen::VectorXd & q_init, const std::vector<Footstep> & input_footstep_list);

    // Single hand manipulation only
    bool computeConfigurationTrajectory(std::shared_ptr<ManipulationFunction> f_s, int robot_manipulation_side, 
    									double s_o, double delta_s, 
    									const Eigen::VectorXd & q_init, const std::vector<Footstep> & input_footstep_list);



    // returns N_size
    int getDiscretizationSize();

    // Sets the SE3 trajectories for the left and right hands
    void setLeftHandTrajectory(const TrajSE3 & traj_SE3_left_hand_in);
    void setRightHandTrajectory(const TrajSE3 & traj_SE3_right_hand_in);    

    // TODO: Populates a constant right and left hand trajectories to be used. 
    void setConstantRightHandTrajectory(const Eigen::Vector3d & des_pos, const Eigen::Quaterniond & des_quat);
    void setConstantLeftHandTrajectory(const Eigen::Vector3d & des_pos, const Eigen::Quaterniond & des_quat);

    // Print Trajectory Result
    void printIntermediateIKTrajectoryresult(int & index, bool & primary_task_converge_result, double & total_error_norm, std::vector<double> & task_error_norms_in);
    // Prints the final trajectory result
    void printIKTrajectoryresult();

	// Set whether to attempt to do a full trajectory IK even with partial divergence during the IK trajectory generation process.
 	// If true: attempt to find a configuration for all task trajectories even if some configurations have big errors
 	// if false: the ik trajectory solver stops at the latest good configuration and sets that configuration for the remainder of the trajectory
	// Default = false.
    void setSolveEvenWithPartialDivergence(bool solve_with_partial_divergence_in);

    // Public Member Variables
	std::shared_ptr<RobotModel> robot_model;

    std::shared_ptr<IKModule> ik_starting_config_module;
    std::shared_ptr<IKModule> ik_locomanipulation_module; 
    std::shared_ptr<IKModule> ik_manipulation_only_module; 

    std::shared_ptr<IKModule> ik_to_use_module;

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
	std::shared_ptr<Task> task_stack_manip_1;
	std::shared_ptr<Task> task_stack_manip_2;

	std::shared_ptr<Task> task_stack_posture_config;
	std::shared_ptr<Task> task_stack_starting_config;


	// Trajectory containers
	TrajEuclidean   traj_q_config; 	// Trajectory of configurations q
	TrajSE3         traj_SE3_left_hand; // Left Hand Trajectories (if it exists)
	TrajSE3         traj_SE3_right_hand; // Right Hand Trajectories (if it exists)

	Eigen::VectorXd q_start;
	Eigen::VectorXd q_current;

	int N_size = 100;

	// Task error norms
	std::vector<double> task_error_norms;


	// Store full IK data?

	// Checks whether the latest trajectory converged.
	// convergence condition:  max_first_task_ik_error < ik_module.error_tol.
	bool didTrajectoryConverge();

	// Trajectory task space error tolerance
	void setTrajErrorTol(double error_tol_in);

	// Set Manipulation Time when there are no footsteps being created
	void setManipulationOnlyTime(double manipulation_only_time_in);

private:
	void initializeIKModules();
	void createTaskStack();

	// Set the task reference using the input configuration q_config. Note that q_config contains the configuration of the entire robot.
	void setPostureTaskReference(std::shared_ptr<Task> & posture_task, const Eigen::VectorXd & q_config);

	// Reference will use the initial joint configuration in traj_q_config.
	void getSelectedPostureTaskReferences(std::vector<std::string> & selected_names, const Eigen::VectorXd & q_config, Eigen::VectorXd & q_ref);

	bool use_right_hand = false;
	bool use_left_hand = false;
	bool use_torso_joint_position = true;

	
	Eigen::Quaterniond tmp_pelvis_ori;
	Eigen::Vector3d tmp_pelvis_pos;

	Eigen::Vector3d tmp_com_pos;	

	Footstep tmp_left_foot;
	Footstep tmp_right_foot;

	Eigen::Vector3d tmp_rhand_pos;
	Eigen::Quaterniond tmp_rhand_ori;

	Eigen::Vector3d tmp_lhand_pos;
	Eigen::Quaterniond tmp_lhand_ori;

	Eigen::VectorXd tmp_torso_posture;
	Eigen::VectorXd tmp_neck_posture;
	Eigen::VectorXd tmp_rarm_posture;
	Eigen::VectorXd tmp_larm_posture;

	int verbosity_level = CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2;

	double max_first_task_ik_error = -1e3;
	double max_manipulation_task_ik_error = 1e-2;

	bool solve_with_partial_divergence = false;

	double traj_error_tol = 1e-2;

	double manipulation_only_time = 3.0;

};


#endif