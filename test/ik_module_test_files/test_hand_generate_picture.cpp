#include <iostream>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>

#include <avatar_locomanipulation/tasks/task.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose_wrt_midfeet.hpp>
#include <avatar_locomanipulation/tasks/task_6dcontact_normal.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>
#include <avatar_locomanipulation/tasks/task_4dcontact_normal.hpp>
#include <avatar_locomanipulation/tasks/task_contact_normal.hpp>

#include <avatar_locomanipulation/tasks/task_6dpose_wrt_frame.hpp>


// Test the stance generation
#include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>
#include "pinocchio/utils/timer.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


// Stance Generation
#include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>
#include <avatar_locomanipulation/data_types/manipulation_function.hpp>


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



struct PositionBool{
	Eigen::Vector3d pos;
	bool b;
};




void visualizeResults(std::vector<PositionBool> & pb, visualization_msgs::MarkerArray & msg){
	visualization_msgs::Marker temp;

	std::cout << "pb.size(): " << pb.size() << std::endl;

	for(int i=0; i<pb.size(); ++i){

		// Fill the Marker 
		temp.pose.position.x = pb[i].pos[0];
		temp.pose.position.y = pb[i].pos[1];
		temp.pose.position.z = pb[i].pos[2];	
		temp.pose.orientation.x = 0.;	
		temp.pose.orientation.y = 0.;
		temp.pose.orientation.z = 0.;
		temp.pose.orientation.w = 1.;
		temp.ns = "foot_positions";
		temp.id = i;
		temp.scale.x = 0.05;
		temp.scale.y = 0.05;
		temp.scale.z = 0.02;
		temp.header.frame_id = "world";
		if(pb[i].b == true){
			temp.color.r = 0.0f;
			temp.color.b = 0.0f;
			temp.color.g = 1.0f;
		}else{
			temp.color.r = 1.0f;
			temp.color.b = 0.0f;
			temp.color.g = 0.0f;
		}
		temp.color.a = 1.0;
		temp.action = visualization_msgs::Marker::ADD;
		temp.type = visualization_msgs::Marker::CUBE;
		msg.markers.push_back(temp);
	}
}


void useIK_module(){
	Eigen::VectorXd q_init;
  initialize_config(q_init);

  // Create IK Module
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(urdf_filename, meshDir_, srdf_filename));

  IKModule ik_module(robot_model);
  ik_module.setInitialConfig(q_init);  
  // Update Robot Kinematics
  ik_module.robot_model->updateFullKinematics(q_init);

  Eigen::Vector3d floor_normal(0,0,1);
  Eigen::Vector3d floor_center(0,0,0);  

  // Create Tasks
  std::shared_ptr<Task> pelvis_task(new Task6DPose(ik_module.robot_model, "pelvis"));

  std::shared_ptr<Task> lfoot_task(new Task6DPose(ik_module.robot_model, "leftCOP_Frame"));
  std::shared_ptr<Task> rfoot_task(new Task6DPose(ik_module.robot_model, "rightCOP_Frame"));

  std::shared_ptr<Task> rpalm_task(new Task6DPose(ik_module.robot_model, "rightPalm"));

  // posture tasks
  std::vector<std::string> uncontrolled_names = {"torsoYaw", "torsoPitch", "torsoRoll", 
                                             "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", 
                                             "lowerNeckPitch", "neckYaw", "upperNeckPitch"};

  std::shared_ptr<Task> torso_larm_task(new TaskJointConfig(ik_module.robot_model, uncontrolled_names));
  torso_larm_task->setTaskGain(1e-1);

  // Stack Tasks in order of priority
  // Stance Generation Test
  std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {rpalm_task, lfoot_task, rfoot_task, pelvis_task}));

  std::shared_ptr<Task> task_stack_priority_2(new TaskStack(ik_module.robot_model, {torso_larm_task})); 

  // Set desired Pelvis configuration
  Eigen::Vector3d pelvis_des_pos;
  Eigen::Quaternion<double> pelvis_des_quat;  

  pelvis_des_pos[0] = -0.0196036; pelvis_des_pos[1] = 9.63243e-19; pelvis_des_pos[2] = 1.04839;
  pelvis_des_quat.x() = -3.9342e-18; pelvis_des_quat.y() = 1.575e-17; pelvis_des_quat.z() = -3.7987e-18; pelvis_des_quat.w() = 1.;

   // Set desired Foot configuration
  Eigen::Vector3d rfoot_des_pos;
  Eigen::Quaternion<double> rfoot_des_quat;

  rfoot_des_pos[0] = 0.0353236; rfoot_des_pos[1] = -0.132544; rfoot_des_pos[2] = 9.51356e-06;
  rfoot_des_quat.x() = -1.61236e-17; rfoot_des_quat.y() = 1.66533e-16; rfoot_des_quat.z() = 1.94444e-18; rfoot_des_quat.w() = 1.;

  Eigen::Vector3d lfoot_des_pos;
  Eigen::Quaternion<double> lfoot_des_quat;

  lfoot_des_pos[0] = -0.364676; lfoot_des_pos[1] = 0.242866; lfoot_des_pos[2] = 9.51356e-06;
  lfoot_des_quat.x() = 0.; lfoot_des_quat.y() = 0.; lfoot_des_quat.z() = 0.258819; lfoot_des_quat.w() = 0.965926;

  // // Set Desired to current configuration except right palm must have identity orientation
  // Eigen::Vector3d rpalm_des_pos;
  // Eigen::Quaternion<double> rpalm_des_quat;
  // ik_module.robot_model->getFrameWorldPose("rightPalm", rpalm_des_pos, rpalm_des_quat);
  // rpalm_des_pos[0] += 0.35;//0.4;//0.35;//0.25;
  // rpalm_des_pos[1] += 0.25;//0.3;//0.25; 
  // rpalm_des_pos[2] += 0.3; 
  // Eigen::AngleAxis<double> axis_angle;
  // axis_angle.angle() = (M_PI/2.0) + (M_PI/4.0);
  // axis_angle.axis() = Eigen::Vector3d(0, 0, 1.0);
  // rpalm_des_quat = axis_angle;


  // // Set Desired Posture to be close to initial
  // Eigen::VectorXd q_des;

  // Set references ------------------------------------------------------------------------
  pelvis_task->setReference(pelvis_des_pos, pelvis_des_quat);
  lfoot_task->setReference(lfoot_des_pos, lfoot_des_quat);
  rfoot_task->setReference(rfoot_des_pos, rfoot_des_quat);
  rpalm_task->setReference(rpalm_des_pos, rpalm_des_quat);

  // getSelectedPostureTaskReferences(ik_module.robot_model, selected_names, q_init, q_des);
  // posture_task->setReference(q_des);


  // getSelectedPostureTaskReferences(ik_module.robot_model, uncontrolled_names, q_init, q_des);
  // torso_larm_task->setReference(q_des);
  

  // // Check if we can get errors given configuration -----------------------------------------------------------------------------
  // Eigen::VectorXd task_error;
  // lfoot_task->getError(task_error);
  // std::cout << "Left Foot Task Error = " << task_error.transpose() << std::endl;

  // rfoot_task->getError(task_error);
  // std::cout << "Right Foot Task Error = " << task_error.transpose() << std::endl;

  // rpalm_task->getError(task_error);
  // std::cout << "Right Palm Task Error = " << task_error.transpose() << std::endl;

  // posture_task->getError(task_error);
  // std::cout << "Posture Task Error = " << task_error.transpose() << std::endl;
  
  // pelvis_wrt_mf_task->getError(task_error);
  // std::cout << "Pelivs wrt Midfeeet Task Error = " << task_error.transpose() << std::endl;

  // // Add tasks to hierarchy 
  // ik_module.addTasktoHierarchy(task_stack_priority_1);
  // ik_module.addTasktoHierarchy(task_stack_priority_2);
  // // ik_module.addTasktoHierarchy(task_stack_priority_3);



  // // Set backtracking parameters
  // ik_module.setSequentialDescent(false);
  // ik_module.setBackTrackwithCurrentTaskError(true);
  // ik_module.setCheckPrevViolations(true);


  // ik_module.setEnableInertiaWeighting(false);

  // // Perform IK  
  // int solve_result;
  // double total_error_norm;
  // std::vector<double> task_error_norms;
  // Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(ik_module.robot_model->getDimQdot());

  // ik_module.prepareNewIKDataStrcutures();
  // ik_module.solveIK(solve_result, task_error_norms, total_error_norm, q_sol);
  // ik_module.printSolutionResults();

  // // Visualize Solution:
  // std::cout << "Visualizing solution..." << std::endl;
  // visualize_robot(q_init, q_sol);
}


// Once I have the IK solution, I BELIEVE what I do is move the hand fixed position in varying spots as i take that step
// And see if it is possible
// If it is not, then this hand position is red
// If it is, then this hand position is green

int main(int argc, char ** argv){


	
}