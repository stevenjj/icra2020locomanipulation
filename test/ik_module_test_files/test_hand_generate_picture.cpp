// #include <iostream>
// #include <avatar_locomanipulation/ik_module/ik_module.hpp>

// #include <avatar_locomanipulation/tasks/task.hpp>
// #include <avatar_locomanipulation/tasks/task_6dpose.hpp>
// #include <avatar_locomanipulation/tasks/task_6dpose_wrt_midfeet.hpp>
// #include <avatar_locomanipulation/tasks/task_6dcontact_normal.hpp>
// #include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
// #include <avatar_locomanipulation/tasks/task_joint_config.hpp>
// #include <avatar_locomanipulation/tasks/task_stack.hpp>
// #include <avatar_locomanipulation/tasks/task_com.hpp>
// #include <avatar_locomanipulation/tasks/task_4dcontact_normal.hpp>
// #include <avatar_locomanipulation/tasks/task_contact_normal.hpp>

// #include <avatar_locomanipulation/tasks/task_6dpose_wrt_frame.hpp>


// // Test the stance generation
// #include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>

// // Import ROS and Rviz visualization
// #include <ros/ros.h>
// #include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

// #include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
// #include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>
// #include "pinocchio/utils/timer.hpp"

// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>


// Stance Generation
#include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>
#include <avatar_locomanipulation/data_types/manipulation_function.hpp>

#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>
#include "pinocchio/utils/timer.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

// YAML
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>


// Stance Generation
#include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>
#include <avatar_locomanipulation/data_types/manipulation_function.hpp>

// Standard
#include <iostream>
#include <math.h>
#include <cassert>


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
		temp.ns = "foot_positions";
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
		temp.color.a = 0.9; //1.0;
		temp.action = visualization_msgs::Marker::ADD;
		temp.type = visualization_msgs::Marker::CUBE;
		msg.markers.push_back(temp);
	}


  // for(int i=0; i<pb.size(); ++i){

  //   // Fill the Marker 
  //   temp.pose.position.x = pb[i].pos[0];
  //   temp.pose.position.y = pb[i].pos[1];
  //   temp.pose.position.z = pb[i].pos[2];  
  //   temp.pose.orientation.x = 0.; 
  //   temp.pose.orientation.y = 0.;
  //   temp.pose.orientation.z = 0.;
  //   temp.pose.orientation.w = 1.;
  //   temp.ns = "foot_positions";
  //   temp.id = i;
  //   temp.scale.x = 0.02;
  //   temp.scale.y = 0.05;
  //   temp.scale.z = 0.05;
  //   temp.header.frame_id = "world";
  //   if(pb[i].b == true){
  //     temp.color.r = 0.0f;
  //     temp.color.b = 0.0f;
  //     temp.color.g = 1.0f;
  //   }else{
  //     temp.color.r = 1.0f;
  //     temp.color.b = 0.0f;
  //     temp.color.g = 0.0f;
  //   }
  //   temp.color.a = 1.0;
  //   temp.action = visualization_msgs::Marker::ADD;
  //   temp.type = visualization_msgs::Marker::CUBE;
  //   msg.markers.push_back(temp);
  // }
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


// void useIK_module(Eigen::VectorXd & q_){
	
//   // Create IK Module
//   std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
//   std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
//   std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
//   std::shared_ptr<RobotModel> robot_model(new RobotModel(urdf_filename, meshDir_, srdf_filename));

//   Eigen::VectorXd q_init;
//   initialize_config(q_init, robot_model);

//   IKModule ik_module(robot_model);
//   ik_module.setInitialConfig(q_init);  
//   // Update Robot Kinematics
//   ik_module.robot_model->updateFullKinematics(q_init);

//   Eigen::Vector3d floor_normal(0,0,1);
//   Eigen::Vector3d floor_center(0,0,0);  

//   // Create Tasks
//   std::shared_ptr<Task> pelvis_task(new Task6DPose(ik_module.robot_model, "pelvis"));

//   std::shared_ptr<Task> lfoot_task(new Task6DPose(ik_module.robot_model, "leftCOP_Frame"));
//   std::shared_ptr<Task> rfoot_task(new Task6DPose(ik_module.robot_model, "rightCOP_Frame"));

//   std::shared_ptr<Task> rpalm_task(new Task6DPose(ik_module.robot_model, "rightPalm"));

//   // posture tasks
//   std::vector<std::string> uncontrolled_names = {"torsoYaw", "torsoPitch", "torsoRoll", 
//                                              "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", 
//                                              "lowerNeckPitch", "neckYaw", "upperNeckPitch"};

//   std::shared_ptr<Task> torso_larm_task(new TaskJointConfig(ik_module.robot_model, uncontrolled_names));
//   torso_larm_task->setTaskGain(1e-1);

//   // Stack Tasks in order of priority
//   // Stance Generation Test
//   std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {rpalm_task, lfoot_task, rfoot_task, pelvis_task}));

//   std::shared_ptr<Task> task_stack_priority_2(new TaskStack(ik_module.robot_model, {torso_larm_task})); 

//   // Set desired Pelvis configuration
//   Eigen::Vector3d pelvis_des_pos;
//   Eigen::Quaternion<double> pelvis_des_quat;  

//   pelvis_des_pos[0] = -0.0196036; pelvis_des_pos[1] = 9.63243e-19; pelvis_des_pos[2] = 1.04839;
//   pelvis_des_quat.x() = -3.9342e-18; pelvis_des_quat.y() = 1.575e-17; pelvis_des_quat.z() = -3.7987e-18; pelvis_des_quat.w() = 1.;

//    // Set desired Foot configuration
//   Eigen::Vector3d rfoot_des_pos;
//   Eigen::Quaternion<double> rfoot_des_quat;

//   rfoot_des_pos[0] = -0.364676; rfoot_des_pos[1] = -0.132544; rfoot_des_pos[2] = 9.51356e-06;
//   rfoot_des_quat.x() = -1.61236e-17; rfoot_des_quat.y() = 1.66533e-16; rfoot_des_quat.z() = 1.94444e-18; rfoot_des_quat.w() = 1.;

//   Eigen::Vector3d lfoot_des_pos;
//   Eigen::Quaternion<double> lfoot_des_quat;

//   lfoot_des_pos[0] = -0.364676; lfoot_des_pos[1] = 0.242866; lfoot_des_pos[2] = 9.51356e-06;
//   lfoot_des_quat.x() = 0.; lfoot_des_quat.y() = 0.; lfoot_des_quat.z() = 0.258819; lfoot_des_quat.w() = 0.965926;

//   // Set Desired to current configuration except right palm must have identity orientation
//   Eigen::Vector3d rpalm_des_pos;
//   Eigen::Quaternion<double> rpalm_des_quat;

//   rpalm_des_pos[0] = 0.280527; rpalm_des_pos[1] = -0.378868; rpalm_des_pos[2] = 1.02383;
//   rpalm_des_quat.x() = 0.510743; rpalm_des_quat.y() = -0.483792; rpalm_des_quat.z() = 0.494538; rpalm_des_quat.w() = 0.510411;

//   // Set Desired Posture to be close to initial
//   Eigen::VectorXd q_des;

//   // Set references ------------------------------------------------------------------------
//   pelvis_task->setReference(pelvis_des_pos, pelvis_des_quat);
//   lfoot_task->setReference(lfoot_des_pos, lfoot_des_quat);
//   rfoot_task->setReference(rfoot_des_pos, rfoot_des_quat);
//   rpalm_task->setReference(rpalm_des_pos, rpalm_des_quat);

//   // getSelectedPostureTaskReferences(ik_module.robot_model, selected_names, q_init, q_des);
//   // posture_task->setReference(q_des);


//   getSelectedPostureTaskReferences(ik_module.robot_model, uncontrolled_names, q_init, q_des);
//   torso_larm_task->setReference(q_des);
  

//   // Check if we can get errors given configuration -----------------------------------------------------------------------------
//   Eigen::VectorXd task_error;
//   lfoot_task->getError(task_error);
//   std::cout << "Left Foot Task Error = " << task_error.transpose() << std::endl;

//   rfoot_task->getError(task_error);
//   std::cout << "Right Foot Task Error = " << task_error.transpose() << std::endl;

//   rpalm_task->getError(task_error);
//   std::cout << "Right Palm Task Error = " << task_error.transpose() << std::endl;

//   // Add tasks to hierarchy 
//   ik_module.addTasktoHierarchy(task_stack_priority_1);
//   ik_module.addTasktoHierarchy(task_stack_priority_2);

//   // Set backtracking parameters
//   ik_module.setSequentialDescent(false);
//   ik_module.setBackTrackwithCurrentTaskError(true);
//   ik_module.setCheckPrevViolations(true);


//   ik_module.setEnableInertiaWeighting(false);

//   // Perform IK  
//   int solve_result;
//   double total_error_norm;
//   std::vector<double> task_error_norms;
//   Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(ik_module.robot_model->getDimQdot());

//   ik_module.prepareNewIKDataStrcutures();
//   ik_module.solveIK(solve_result, task_error_norms, total_error_norm, q_sol);
//   ik_module.printSolutionResults();

//   q_ = q_sol;

// }


// Once I have the IK solution, I BELIEVE what I do is move the hand fixed position in varying spots as i take that step
// And see if it is possible
// If it is not, then this hand position is red
// If it is, then this hand position is green


void test_hand_in_place_config_trajectory_generator(){
  std::cout << "[Running Config Trajectory Generator Test] In place walking with hand constant" << std::endl;

  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  int N_resolution = 60; 
  ConfigTrajectoryGenerator ctg(valkyrie_model, N_resolution);

  Eigen::VectorXd q_start, q_end;
  initialize_config(q_start, valkyrie_model);

  // Test initial configuration computation for flat ground
  ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_1);
  ctg.computeInitialConfigForFlatGround(q_start, q_end);

  // Create visualization object
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(ros_node, valkyrie_model);  

  // Update Initial
  q_start = q_end;
  valkyrie_model->updateFullKinematics(q_start);

  // In place step test with keeping the hand in place
  // Create footsteps in place
  Footstep footstep_1; footstep_1.setLeftSide();
  Footstep footstep_2; footstep_2.setRightSide();

  valkyrie_model->getFrameWorldPose("leftCOP_Frame", footstep_1.position, footstep_1.orientation);  
  valkyrie_model->getFrameWorldPose("rightCOP_Frame", footstep_2.position, footstep_2.orientation);  

  double theta = M_PI/6.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Quaternion<double> quat_angleaxis(aa); //Initialized to remember the w component comes first

  double x_min = -0.3;
  double x_max = 0.2;
  double y_min = -0.15;
  double y_max = 0.1;
  double z_min = -0.1;
  double z_max = 0.15;
  double dx = 0.05;
  int N = static_cast<int>((x_max - x_min) / dx);
  int M = static_cast<int>((y_max - y_min) / dx);
  int L = static_cast<int>((z_max - z_min) / dx);
  double x_, y_, z_;

  PositionBool temp;
  std::vector<PositionBool> pb;
  ros::Rate rate(1000);
  ros::NodeHandle n;

  ros::Publisher hand_markers_pub = n.advertise<visualization_msgs::MarkerArray>("foot_positions", 100);

  visualization_msgs::MarkerArray msg;

  Eigen::Vector3d original_pos; Eigen::Quaternion<double> original_ori;

  valkyrie_model->getFrameWorldPose("rightPalm", original_pos, original_ori);  

  Eigen::Vector3d rhand_pos; Eigen::Quaternion<double> rhand_ori;

  footstep_1.orientation.x() = 0.; footstep_1.orientation.y() = 0.; footstep_1.orientation.z() = 0.258819; footstep_1.orientation.w() = 0.965926;

  // Add this current x_ to the foot pos
  footstep_1.position[0] -= 0.4;
  // Get y pos at this step
  // Add this current y_ to the foot pos
  footstep_1.position[1] += 0.1;



  for(int i=0; i<N+1; ++i){ 
    // Get x pos at this step
    x_ = x_min + i*dx;
  // for(int i=0; i<L+1; ++i){
  //   // Get z pos at this step
  //   z_ = z_min + i*dx;

    for(int j=0; j <M+1; ++j){
      // Get y pos at this step
      y_ = y_min + j*dx;

      footstep_1.setPosOri(footstep_1.position, footstep_1.orientation);
      footstep_2.setPosOri(footstep_2.position, quat_angleaxis);


      std::vector<Footstep> input_footstep_list = {footstep_1};

      // Reset the hand pose
      rhand_pos = original_pos; 
      rhand_ori = original_ori;

      // // Add this current x_ to the foot pos
      // rhand_pos[0] += x_;
      rhand_pos[0] += x_;
      // // Add this current z_ to the foot pos
      // rhand_pos[2] += z_;
      // Add this current y_ to the foot pos
      rhand_pos[1] += y_;

      // Set Constant Right Hand trajectory to current //To do. set task gain for hand to be small in orientation
      ctg.setConstantRightHandTrajectory(rhand_pos, rhand_ori);
      ctg.setUseRightHand(true);
      ctg.setUseTorsoJointPosition(false);
      ctg.reinitializeTaskStack();

        // timer
      PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);


      // Have fast double support times
      ctg.wpg.setDoubleSupportTime(0.2);
      // Set Verbosity
      ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2);
      // Solve for configurations
      timer.tic();
      temp.b = ctg.computeConfigurationTrajectory(q_start, input_footstep_list);
      temp.pos = rhand_pos;
      pb.push_back(temp);
      std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

      visualizeResults(pb, msg);
      hand_markers_pub.publish(msg);
      rate.sleep();
      ros::spinOnce();

      // Visualize Trajectory
      visualizer.visualizeConfigurationTrajectory(q_start, ctg.traj_q_config, true);
      
    }
  }  

  footstep_1.setPosOri(footstep_1.position, quat_angleaxis);
  footstep_2.setPosOri(footstep_2.position, quat_angleaxis);

  std::vector<Footstep> input_footstep_list = {footstep_1};

  // Reset the hand pose
  rhand_pos = original_pos; 
  rhand_ori = original_ori;

  // Add this current x_ to the foot pos
  rhand_pos[0] -= 0.1;
  // Add this current y_ to the foot pos
  rhand_pos[1] -= 0.05;


  // Set Constant Right Hand trajectory to current //To do. set task gain for hand to be small in orientation
  ctg.setConstantRightHandTrajectory(rhand_pos, rhand_ori);
  ctg.setUseRightHand(true);
  ctg.setUseTorsoJointPosition(false);
  ctg.reinitializeTaskStack();

    // timer
  PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);


  // Have fast double support times
  ctg.wpg.setDoubleSupportTime(0.2);
  // Set Verbosity
  ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2);
  // Solve for configurations
  timer.tic();
  temp.b = ctg.computeConfigurationTrajectory(q_start, input_footstep_list);
  temp.pos = rhand_pos;
  pb.push_back(temp);
  std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  visualizeResults(pb, msg);
  hand_markers_pub.publish(msg);
  rate.sleep();
  ros::spinOnce();

  // Visualize Trajectory
  visualizer.visualizeConfigurationTrajectory(q_start, ctg.traj_q_config, true);

////////////////////////// AFTER THIS WAS ADDDED FOR ROSBAG

  ros::Rate loop_rate(20);
  Eigen::VectorXd q_finish;
  ctg.traj_q_config.get_pos(ctg.getDiscretizationSize() - 1, q_finish);

  // Joint State Publisher
  ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
  ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);
  // Initialize Rviz translator
  ValRvizTranslator rviz_translator;

  // Transform broadcaster
  tf::TransformBroadcaster      br_ik;
  tf::TransformBroadcaster      br_robot;

  // Initialize Transforms and Messages
  tf::Transform tf_world_pelvis_init;
  tf::Transform tf_world_pelvis_end;

  sensor_msgs::JointState joint_msg_init;
  sensor_msgs::JointState joint_msg_end;

  // Visualize q_start and q_end in RVIZ
  rviz_translator.populate_joint_state_msg(valkyrie_model->model, q_start, tf_world_pelvis_init, joint_msg_init);
  rviz_translator.populate_joint_state_msg(valkyrie_model->model, q_finish, tf_world_pelvis_end, joint_msg_end);

  while(ros::ok()){
    hand_markers_pub.publish(msg);

    br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
    robot_joint_state_pub.publish(joint_msg_init);

    br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
    robot_ik_joint_state_pub.publish(joint_msg_end);
    ros::spinOnce();
    loop_rate.sleep();

  }

}



int main(int argc, char ** argv){

  ros::init(argc, argv, "test_footstep_probability");

  // useIK_module(q_sol);

  test_hand_in_place_config_trajectory_generator();
  
	
}