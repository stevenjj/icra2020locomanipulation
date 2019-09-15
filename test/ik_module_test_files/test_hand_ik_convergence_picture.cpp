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
#include <avatar_locomanipulation/tasks/task_xdpose.hpp>


#include <avatar_locomanipulation/tasks/task_6dpose_wrt_frame.hpp>


// Test the stance generation
#include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


void initialize_config(Eigen::VectorXd & q_init, std::shared_ptr<RobotModel> & valkyrie){

  // X dimensional state vectors
  Eigen::VectorXd q_start;

  // std::cout << "valkyrie->getDimQ(): " << valkyrie->getDimQ() << std::endl;
  // Set origin at 0.0
  q_start = Eigen::VectorXd::Zero(valkyrie->getDimQ());

  double theta = 0.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q
  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[valkyrie->getJointIndex("leftHipPitch")] = -0.3;
  q_start[valkyrie->getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie->getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie->getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie->getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie->getJointIndex("rightAnklePitch")] = -0.3;

  q_start[valkyrie->getJointIndex("rightShoulderPitch")] = 0.07;
  q_start[valkyrie->getJointIndex("rightShoulderRoll")] = 1.19;
  q_start[valkyrie->getJointIndex("rightElbowPitch")] = 1.6 ; //0.4;
  q_start[valkyrie->getJointIndex("rightForearmYaw")] = -0.37;

  q_start[valkyrie->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie->getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}



struct PositionBool{
	Eigen::Vector3d pos;
	bool b;
};


void visualizeResults(std::vector<PositionBool> & pb, visualization_msgs::MarkerArray & msg){
	visualization_msgs::Marker temp;

	std::cout << "pb.size(): " << pb.size() << std::endl;
	double factor = 0.95;

	for(int i=0; i<pb.size(); ++i){

		// Fill the Marker 
		temp.pose.position.x = pb[i].pos[0];
		temp.pose.position.y = pb[i].pos[1];
		temp.pose.position.z = pb[i].pos[2];	
		temp.pose.orientation.x = 0.;	
		temp.pose.orientation.y = 0.;
		temp.pose.orientation.z = 0.;
		temp.pose.orientation.w = 1.;
		temp.ns = "hand_positions";
		temp.id = i;
		temp.scale.x = 0.05*factor;
		temp.scale.y = 0.05*factor;
		temp.scale.z = 0.02*factor;
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


void getSelectedPostureTaskReferences(std::shared_ptr<RobotModel> valkyrie, std::vector<std::string> & selected_names, Eigen::VectorXd & q_start, Eigen::VectorXd & q_ref){
  Eigen::VectorXd q_des;
  q_des = Eigen::VectorXd::Zero(selected_names.size());
  for(int i = 0; i < selected_names.size(); i++){
    std::cout << selected_names[i] << std::endl;
    q_des[i] = q_start[valkyrie->getJointIndex(selected_names[i])];
  }
  q_ref = q_des;
}


void visualize_robot(Eigen::VectorXd & q_start, Eigen::VectorXd & q_end, visualization_msgs::MarkerArray & msg){
  // Initialize ROS node for publishing joint messages
  ros::NodeHandle n;
  ros::Rate loop_rate(20);

  // Initialize Rviz translator
  ValRvizTranslator rviz_translator;

  // Transform broadcaster
  tf::TransformBroadcaster      br_ik;
  tf::TransformBroadcaster      br_robot;
  // Joint State Publisher
  ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
  ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);

  // Initialize Transforms and Messages
  tf::Transform tf_world_pelvis_init;
  tf::Transform tf_world_pelvis_end;

  sensor_msgs::JointState joint_msg_init;
  sensor_msgs::JointState joint_msg_end;

  // Initialize Robot Model
  std::cout << "Initialize Valkyrie Model" << std::endl;
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  RobotModel valkyrie(urdf_filename, meshDir_, srdf_filename);

  // Visualize q_start and q_end in RVIZ
  rviz_translator.populate_joint_state_msg(valkyrie.model, q_start, tf_world_pelvis_init, joint_msg_init);
  rviz_translator.populate_joint_state_msg(valkyrie.model, q_end, tf_world_pelvis_end, joint_msg_end);

  ros::Publisher hand_pub = n.advertise<visualization_msgs::MarkerArray>("foot_positions", 100);

  while(ros::ok()){
      br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
      robot_joint_state_pub.publish(joint_msg_init);

      br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
      robot_ik_joint_state_pub.publish(joint_msg_end);

      hand_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}


void useIK_module(){
	
  // Create IK Module
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(urdf_filename, meshDir_, srdf_filename));

  Eigen::VectorXd q_init;
  initialize_config(q_init, robot_model);

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

  // std::shared_ptr<Task> rpalm_task(new TaskXDPose(ik_module.robot_model, {TASK_DIM_X, TASK_DIM_Y, TASK_DIM_Z}, "rightPalm"));
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

  ik_module.robot_model->getFrameWorldPose("pelvis", pelvis_des_pos, pelvis_des_quat);

   // Set desired Foot configuration
  Eigen::Vector3d rfoot_des_pos;
  Eigen::Quaternion<double> rfoot_des_quat;

  ik_module.robot_model->getFrameWorldPose("rightCOP_Frame", rfoot_des_pos, rfoot_des_quat);

  Eigen::Vector3d lfoot_des_pos;
  Eigen::Quaternion<double> lfoot_des_quat;

  ik_module.robot_model->getFrameWorldPose("leftCOP_Frame", lfoot_des_pos, lfoot_des_quat);

  // Set Desired Posture to be close to initial
  Eigen::VectorXd q_des;

  // Set references ------------------------------------------------------------------------
  pelvis_task->setReference(pelvis_des_pos, pelvis_des_quat);
  lfoot_task->setReference(lfoot_des_pos, lfoot_des_quat);
  rfoot_task->setReference(rfoot_des_pos, rfoot_des_quat);

  getSelectedPostureTaskReferences(ik_module.robot_model, uncontrolled_names, q_init, q_des);
  torso_larm_task->setReference(q_des);

  // Add tasks to hierarchy 
  ik_module.addTasktoHierarchy(task_stack_priority_1);
  ik_module.addTasktoHierarchy(task_stack_priority_2);

  // Set backtracking parameters
  ik_module.setSequentialDescent(false);
  ik_module.setBackTrackwithCurrentTaskError(true);
  ik_module.setCheckPrevViolations(true);

  ik_module.setEnableInertiaWeighting(false);

  double x_min = -0.3;
  double x_max = 0.4;
  double y_min = -0.4;
  double y_max = 0.2;
  double dx = 0.05;
  int N = static_cast<int>((x_max - x_min) / dx);
  int M = static_cast<int>((y_max - y_min) / dx);
  double x_, y_;

  Eigen::Vector3d rpalm_des_pos, original_pos;
  Eigen::Quaternion<double> rpalm_des_quat;

  ik_module.robot_model->getFrameWorldPose("rightPalm", original_pos, rpalm_des_quat);

  int solve_result;
  double total_error_norm;
  std::vector<double> task_error_norms;
  Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(ik_module.robot_model->getDimQdot());

  PositionBool temp;
	std::vector<PositionBool> pb;
	ros::Rate rate(1000);
	ros::NodeHandle n;

	ros::Publisher hand_markers_pub = n.advertise<visualization_msgs::MarkerArray>("foot_positions", 100);

	visualization_msgs::MarkerArray msg;

  for(int i=0; i<N+1; ++i){
  	// Get x for this step
  	x_ = x_min + i*dx;

  	for(int j=0; j <M+1; ++j){
  		// Get y for this step
  		y_ = y_min + j*dx;
  		// Reset initial config each time
  		ik_module.setInitialConfig(q_init);  
  		// Update Robot Kinematics
  		ik_module.robot_model->updateFullKinematics(q_init);
  		// Reset the right hand position
  		rpalm_des_pos = original_pos;

  		// Update rhand des pos for each step
  		rpalm_des_pos[0] += x_;
  		rpalm_des_pos[1] += y_;
  		// Set task reference
  		rpalm_task->setReference(rpalm_des_pos, rpalm_des_quat);

  		// Perform IK  
			ik_module.prepareNewIKDataStrcutures();
			temp.b = ik_module.solveIK(solve_result, task_error_norms, total_error_norm, q_sol);
			temp.pos = rpalm_des_pos;
			ik_module.printSolutionResults();

			pb.push_back(temp);

			visualizeResults(pb, msg);
  		hand_markers_pub.publish(msg);
			rate.sleep();
			ros::spinOnce();

			// Visualize Trajectory
  		// visualize_robot(q_init, q_sol);
  	}
  }

  // Add this current x_ to the foot pos
  x_ = -0.2;
  // Add this current y_ to the foot pos
  y_ = 0.05;

  // Reset initial config each time
	ik_module.setInitialConfig(q_init);  
	// Update Robot Kinematics
	ik_module.robot_model->updateFullKinematics(q_init);
	// Reset the right hand position
	rpalm_des_pos = original_pos;

	// Update rhand des pos for each step
	rpalm_des_pos[0] += x_;
	rpalm_des_pos[1] += y_;
	// Set task reference
	rpalm_task->setReference(rpalm_des_pos);

	// Perform IK  
	ik_module.prepareNewIKDataStrcutures();
	temp.b = ik_module.solveIK(solve_result, task_error_norms, total_error_norm, q_sol);
	temp.pos = rpalm_des_pos;
	ik_module.printSolutionResults();

	pb.push_back(temp);

	visualizeResults(pb, msg);
	hand_markers_pub.publish(msg);
	rate.sleep();
	ros::spinOnce();

	// Visualize Trajectory
	visualize_robot(q_init, q_sol, msg);

}


int main(int argc, char ** argv){
	ros::init(argc, argv, "test_footstep_probability");

	ros::Time::init();

	useIK_module();
}