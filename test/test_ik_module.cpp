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


// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>


void visualize_robot(Eigen::VectorXd & q_start, Eigen::VectorXd & q_end);

void printVec(const std::string & vec_name, const::Eigen::VectorXd vec){
  std::cout << vec_name << ": ";
  for(int i = 0; i < vec.size(); i++){
    std::cout << vec[i] << ", ";
  } 
  std::cout << std::endl;
}

void initialize_config(Eigen::VectorXd & q_init){

  std::cout << "Initialize Valkyrie Model" << std::endl;
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  RobotModel valkyrie(urdf_filename, meshDir_, srdf_filename);

  std::cout << "test_ik s1" << std::endl;
  // X dimensional state vectors
  Eigen::VectorXd q_start;

  std::cout << "valkyrie.getDimQ(): " << valkyrie.getDimQ() << std::endl;
  // Set origin at 0.0
  q_start = Eigen::VectorXd::Zero(valkyrie.getDimQ());
  std::cout << "test_ik s2" << std::endl;
  double theta = 0.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q

  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location
  std::cout << "test_ik s3" << std::endl;

  q_start[valkyrie.getJointIndex("leftHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightAnklePitch")] = 0.0;//-0.3;

  q_start[valkyrie.getJointIndex("rightShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie.getJointIndex("rightElbowPitch")] = 0.4;
  q_start[valkyrie.getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}

void getPostureTaskReferences(std::shared_ptr<RobotModel> valkyrie, Eigen::VectorXd & q_start, Eigen::VectorXd & q_ref){
  Eigen::VectorXd q_des;
  q_des = Eigen::VectorXd::Zero(valkyrie->joint_names.size());

  for(int i = 0; i < valkyrie->joint_names.size(); i++){
    std::cout << valkyrie->joint_names[i] << std::endl;
    q_des[i] = q_start[valkyrie->getJointIndex(valkyrie->joint_names[i])];
  }
  q_des[valkyrie->getJointIndexNoFloatingJoints("leftElbowPitch")] = -1.25;
  std::cout << "q_start = " << q_start.transpose() << std::endl;
  std::cout << "q_des = " << q_des.transpose() << std::endl;
  q_ref = q_des;
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


void testIK_module(){
  std::cout << "[IK Module Test]" << std::endl;
  Eigen::VectorXd q_init;
  initialize_config(q_init);

  // Create IK Module
  IKModule ik_module;
  ik_module.setInitialConfig(q_init);  
  // Update Robot Kinematics
  ik_module.robot_model->updateFullKinematics(q_init);

  Eigen::Vector3d floor_normal(0,0,1);
  Eigen::Vector3d floor_center(0,0,0);  

  // Create Tasks
  std::shared_ptr<Task> pelvis_task(new Task6DPose(ik_module.robot_model, "pelvis"));
  std::shared_ptr<Task> pelvis_wrt_mf_task(new Task6DPosewrtMidFeet(ik_module.robot_model, "pelvis"));

  std::shared_ptr<Task> lfoot_task(new Task6DPose(ik_module.robot_model, "leftCOP_Frame"));
  std::shared_ptr<Task> rfoot_task(new Task6DPose(ik_module.robot_model, "rightCOP_Frame"));

  std::shared_ptr<Task> lfoot_contact_task(new Task4DContactNormalTask(ik_module.robot_model, "leftCOP_Frame", floor_normal, floor_center));
  std::shared_ptr<Task> rfoot_contact_task(new Task4DContactNormalTask(ik_module.robot_model, "rightCOP_Frame", floor_normal, floor_center));

  std::shared_ptr<Task> lfoot_contact_normal_task(new TaskContactNormalTask(ik_module.robot_model, "leftCOP_Frame", floor_normal, floor_center));
  std::shared_ptr<Task> rfoot_contact_normal_task(new TaskContactNormalTask(ik_module.robot_model, "rightCOP_Frame", floor_normal, floor_center));

  std::shared_ptr<Task> com_task(new TaskCOM(ik_module.robot_model));
  std::shared_ptr<Task> rpalm_task(new Task6DPose(ik_module.robot_model, "rightPalm"));

  // Left foot w.r.t Right Foot task
  std::shared_ptr<Task> lf_wrt_rf_task(new Task6DPosewrtFrame(ik_module.robot_model, "leftCOP_Frame", "rightCOP_Frame"));


  // posture tasks
  std::vector<std::string> uncontrolled_names = {"torsoYaw", "torsoPitch", "torsoRoll", 
                                             "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", 
                                             "lowerNeckPitch", "neckYaw", "upperNeckPitch"};
  std::shared_ptr<Task> torso_larm_task(new TaskJointConfig(ik_module.robot_model, uncontrolled_names));
  torso_larm_task->setTaskGain(1e-1);

  // std::vector<std::string> selected_names = {"rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch"};
 std::vector<std::string> selected_names = ik_module.robot_model->joint_names;
  std::shared_ptr<Task> posture_task(new TaskJointConfig(ik_module.robot_model, selected_names));
  posture_task->setTaskGain(1e-1);

  // Stack Tasks in order of priority
  // Stance Generation Test
  std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {rpalm_task, lfoot_contact_normal_task, rfoot_contact_normal_task, pelvis_wrt_mf_task, torso_larm_task}));
  // std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {rpalm_task, lfoot_contact_task, rfoot_contact_task}));
  std::shared_ptr<Task> task_stack_priority_2(new TaskStack(ik_module.robot_model, {lf_wrt_rf_task, posture_task})); 
  // std::shared_ptr<Task> task_stack_priority_2(new TaskStack(ik_module.robot_model, {lfoot_contact_task, rfoot_contact_task}));

  // std::shared_ptr<Task> task_stack_priority_3(new TaskStack(ik_module.robot_model, {pelvis_wrt_mf_task, posture_task}));

  // Regular IK test
  // std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {lfoot_task, rfoot_task, pelvis_wrt_mf_task}));
  // std::shared_ptr<Task> task_stack_priority_2(new TaskStack(ik_module.robot_model, {rpalm_task}));
  // std::shared_ptr<Task> task_stack_priority_3(new TaskStack(ik_module.robot_model, {posture_task}));
  // std::shared_ptr<Task> task_stack_priority_3(new TaskStack(ik_module.robot_model, {posture_task, com_task}));


  // Set desired Pelvis configuration
  Eigen::Vector3d pelvis_des_pos;
  Eigen::Quaternion<double> pelvis_des_quat;  

  // Set desired Pelvis configuration wrt midfeet
  Eigen::Vector3d pelvis_wrt_mf_des_pos;
  Eigen::Quaternion<double> pelvis_wrt_mf_des_quat;  

   // Set desired Foot configuration
  Eigen::Vector3d rfoot_des_pos;
  Eigen::Quaternion<double> rfoot_des_quat;

  Eigen::Vector3d lfoot_des_pos;
  Eigen::Quaternion<double> lfoot_des_quat;

  // Axis Angle
  double theta = M_PI/12.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Quaternion<double> des_init_quat(aa);

  // Pelvis should be 1m above the ground and the orientation must be identity
  pelvis_des_pos.setZero();
  pelvis_des_pos[2] = 1.05;
  pelvis_des_quat.setIdentity();

  pelvis_wrt_mf_des_pos.setZero();
  pelvis_wrt_mf_des_pos[2] = 1.05;
  pelvis_wrt_mf_des_quat.setIdentity();

  // Foot should be flat on the ground and spaced out by 0.25m
  rfoot_des_pos.setZero();
  rfoot_des_pos[0] = 0.125;
  rfoot_des_pos[1] = -0.125;
  rfoot_des_quat.setIdentity();

  lfoot_des_pos.setZero();
  lfoot_des_pos[1] = 0.125;
  lfoot_des_quat.setIdentity();
  lfoot_des_quat = des_init_quat;

  // Set Desired to current configuration except right palm must have identity orientation
  Eigen::Vector3d rpalm_des_pos;
  Eigen::Quaternion<double> rpalm_des_quat;
  ik_module.robot_model->getFrameWorldPose("rightPalm", rpalm_des_pos, rpalm_des_quat);
  rpalm_des_pos[0] += 1.35;//0.4;//0.35;//0.25;
  rpalm_des_pos[1] += 0.25;//0.3;//0.25; 
  rpalm_des_pos[2] += 0.3; 
  Eigen::AngleAxis<double> axis_angle;
  axis_angle.angle() = (M_PI/2.0) - (M_PI/4.0);
  axis_angle.axis() = Eigen::Vector3d(0, 0, 1.0);
  rpalm_des_quat = axis_angle;

  // Set Stance by defining left foot w.r.t right foot
  Eigen::Vector3d lf_wrt_rf_des_pos; lf_wrt_rf_des_pos.setZero();
  Eigen::Quaternion<double> lf_wrt_rf_des_quat; 
  lf_wrt_rf_des_pos[0] = -0.1;
  lf_wrt_rf_des_pos[1] = 0.25;
  axis_angle.angle() = (M_PI/6.0);
  axis_angle.axis() = Eigen::Vector3d(0, 0, 1.0);
  lf_wrt_rf_des_quat = axis_angle;  

  // Set desired CoM to be 1m above the ground
  Eigen::Vector3d com_des_pos; com_des_pos.setZero();
  com_des_pos[2] = 1.0;

  // Set Desired Posture to be close to initial
  Eigen::VectorXd q_des;

  // Set references ------------------------------------------------------------------------
  pelvis_task->setReference(pelvis_des_pos, pelvis_des_quat);
  pelvis_wrt_mf_task->setReference(pelvis_wrt_mf_des_pos, pelvis_wrt_mf_des_quat);
  lfoot_task->setReference(lfoot_des_pos, lfoot_des_quat);
  rfoot_task->setReference(rfoot_des_pos, rfoot_des_quat);
  rpalm_task->setReference(rpalm_des_pos, rpalm_des_quat);
  com_task->setReference(com_des_pos);
  lf_wrt_rf_task->setReference(lf_wrt_rf_des_pos, lf_wrt_rf_des_quat);

  getSelectedPostureTaskReferences(ik_module.robot_model, selected_names, q_init, q_des);
  posture_task->setReference(q_des);


  getSelectedPostureTaskReferences(ik_module.robot_model, uncontrolled_names, q_init, q_des);
  torso_larm_task->setReference(q_des);
  

  // Check if we can get errors given configuration -----------------------------------------------------------------------------
  Eigen::VectorXd task_error;
  lfoot_task->getError(task_error);
  std::cout << "Left Foot Task Error = " << task_error.transpose() << std::endl;

  rfoot_task->getError(task_error);
  std::cout << "Right Foot Task Error = " << task_error.transpose() << std::endl;

  rpalm_task->getError(task_error);
  std::cout << "Right Palm Task Error = " << task_error.transpose() << std::endl;

  posture_task->getError(task_error);
  std::cout << "Posture Task Error = " << task_error.transpose() << std::endl;
  
  pelvis_wrt_mf_task->getError(task_error);
  std::cout << "Pelivs wrt Midfeeet Task Error = " << task_error.transpose() << std::endl;

  // Add tasks to hierarchy 
  ik_module.addTasktoHierarchy(task_stack_priority_1);
  ik_module.addTasktoHierarchy(task_stack_priority_2);
  // ik_module.addTasktoHierarchy(task_stack_priority_3);



  // Set backtracking parameters
  ik_module.setSequentialDescent(false);
  ik_module.setBackTrackwithCurrentTaskError(true);
  ik_module.setCheckPrevViolations(false);


  // Perform IK  
  int solve_result;
  double total_error_norm;
  std::vector<double> task_error_norms;
  Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(ik_module.robot_model->getDimQdot());

  ik_module.prepareNewIKDataStrcutures();
  ik_module.solveIK(solve_result, task_error_norms, total_error_norm, q_sol);
  ik_module.printSolutionResults();

  // Visualize Solution:
  visualize_robot(q_init, q_sol);
  
}

void visualize_robot(Eigen::VectorXd & q_start, Eigen::VectorXd & q_end){
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

  while (ros::ok()){
      br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
      robot_joint_state_pub.publish(joint_msg_init);

      br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
      robot_ik_joint_state_pub.publish(joint_msg_end);
    ros::spinOnce();
    loop_rate.sleep();
  }
}



int main(int argc, char ** argv){
  ros::init(argc, argv, "test_ik_module");
  testIK_module();
  return 0;
}