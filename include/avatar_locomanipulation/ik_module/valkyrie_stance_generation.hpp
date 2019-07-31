#ifndef ALM_VALKYRIE_STANCE_GENERATION_H
#define ALM_VALKYRIE_STANCE_GENERATION_H

#include <Configuration.h>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>

// Default Task Types
#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>

// Special Tasks
#include <avatar_locomanipulation/tasks/task_6dpose_wrt_frame.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose_wrt_midfeet.hpp>
#include <avatar_locomanipulation/tasks/task_contact_normal.hpp>
#include <avatar_locomanipulation/tasks/task_xyrzpose_wrt_frame.hpp>

// Task Stack
#include <avatar_locomanipulation/tasks/task_stack.hpp>

// This performs a simple an unconstrained IK with clamping on the joint configurations to find a stance configuration given a desired hand pose/s
class ValkyrieStanceGeneration{
public:
	// Constructors
	ValkyrieStanceGeneration();
	ValkyrieStanceGeneration(std::shared_ptr<RobotModel> & robot_model_in); // Accepts a robot model to use

	// Destructors
	~ValkyrieStanceGeneration();

  void setRobotModel(std::shared_ptr<RobotModel> & robot_model_in);
  void initializeTasks();

  void setStartingConfig(const Eigen::VectorXd & q_start_in);

  // Whether or not there is a desired hand pose
  void setUseRightHand(bool use_right_hand_in);
  void setUseLeftHand(bool use_left_hand_in);

  // Set Desired Tasks
  void setDesiredRightHandPose(const Eigen::Vector3d des_pos, const Eigen::Quaterniond des_quat);
  void setDesiredLeftHandPose(const Eigen::Vector3d des_pos, const Eigen::Quaterniond des_quat);

  // Computes a stance given the desired references.
  bool computeStance(Eigen::VectorXd & q_out);


	// Member Functions
	IKModule stance_ik_module;
  std::shared_ptr<RobotModel> robot_model;

  Eigen::VectorXd q_start;

	// Tasks
  std::shared_ptr<Task> base_pose_task; // Desired Pose of the robot's base

	std::shared_ptr<Task> pelvis_wrt_mf_task;	// Pelvis w.r.t midfeet task
	std::shared_ptr<Task> lfoot_contact_normal_task; // Left Foot Contact Normal Task
	std::shared_ptr<Task> rfoot_contact_normal_task; // Right foot Contact Normal Task
	std::shared_ptr<Task> lfoot_wrt_rfoot_task; // Left Foot w.r.t Right Foot Task

	std::shared_ptr<Task> lpalm_task; // Left Palm Pose Task
	std::shared_ptr<Task> rpalm_task; // Right Palm Pose Task

  std::shared_ptr<Task> torso_neck_arm_posture_task; // Posture task for the torso, neck and unused arms
	std::shared_ptr<Task> overall_posture_task; // Overall robot posture task

	// Task Stacks
	std::shared_ptr<Task> task_stack_priority_1;
	std::shared_ptr<Task> task_stack_priority_2;

  // Desired Base Configuration to be closed to as much as possible
  Eigen::Vector3d base_des_pos;
  Eigen::Quaterniond base_des_quat;  

  // Desired Pelvis configuration wrt midfeet
  Eigen::Vector3d pelvis_wrt_mf_des_pos;
  Eigen::Quaterniond pelvis_wrt_mf_des_quat;  

  // Foot contact centers and normals
  Eigen::Vector3d left_floor_normal;
  Eigen::Vector3d left_floor_center;    
  Eigen::Vector3d right_floor_normal;
  Eigen::Vector3d right_floor_center;    

  // Left Foot w.r.t right foot task
  Eigen::Vector3d lf_wrt_rf_des_pos;
  Eigen::Quaterniond lf_wrt_rf_des_quat; 

  // Desired hand poses
  Eigen::Vector3d lpalm_des_pos;
  Eigen::Quaterniond lpalm_des_quat;

  Eigen::Vector3d rpalm_des_pos;
  Eigen::Quaterniond rpalm_des_quat;

  // Posture task names
  std::vector<std::string> torso_joint_names;
  std::vector<std::string> neck_joint_names;
  std::vector<std::string> left_arm_joint_names;
  std::vector<std::string> right_arm_joint_names;

  std::vector<std::string> torso_neck_arm_posture_task_names;
  std::vector<std::string> overall_posture_task_joint_names;

  void getSelectedPostureTaskReferences(const std::vector<std::string> & selected_names, const Eigen::VectorXd & q_config, Eigen::VectorXd & q_ref);

  // IK Solutions
  int solve_result;
  double total_error_norm;
  std::vector<double> task_error_norms;
  Eigen::VectorXd q_sol;

private:
  void default_initialization();
  void createTaskStack();

  bool use_right_hand = false;
  bool use_left_hand = false;

  Eigen::Vector3d temp_pos;
  Eigen::Quaterniond temp_quat;
  Eigen::AngleAxisd temp_aa;

};

#endif