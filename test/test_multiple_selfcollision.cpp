#include <iostream>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>

#include <avatar_locomanipulation/tasks/task.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>
#include <avatar_locomanipulation/tasks/task_selfcollision.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose_wrt_midfeet.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>

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

  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  std::cout << "Initialize Valkyrie Model" << std::endl;
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
  q_start[valkyrie.getJointIndex("rightHipRoll")] = 0.10;
  q_start[valkyrie.getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("rightKneePitch")] = 0.72;
  q_start[valkyrie.getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;

    // Puts the rightPalm in collision with the pelvis
  q_start[valkyrie.getJointIndex("rightShoulderPitch")] = -0.11;//-0.2;
  q_start[valkyrie.getJointIndex("rightShoulderRoll")] = 1.12;//1.1;
  q_start[valkyrie.getJointIndex("rightShoulderYaw")] = 1.5;//1.1;
  q_start[valkyrie.getJointIndex("rightElbowPitch")] = 1.66; //0.4;
  q_start[valkyrie.getJointIndex("rightForearmYaw")] = 0.0;


  q_start[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}



void testIK_module(){
  std::cout << "[IK Module Test]" << std::endl;
  Eigen::VectorXd q_init;
  initialize_config(q_init);


  // Create IK Module
  IKModule ik_module;

  std::cout << "Test0\n";

  ik_module.setInitialConfig(q_init);  

  std::cout << "Test0.5\n";

  // Update Robot Kinematics
  ik_module.robot_model->enableUpdateGeomOnKinematicsUpdate(true);
  std::cout << "Test0.6\n";
  ik_module.robot_model->updateFullKinematics(q_init);
  std::cout << "Test0.7\n";

  // ik_module.robot_model->printFrameNames();

  // Create the collision environment
  std::shared_ptr<CollisionEnvironment> collision;
  collision = std::shared_ptr<CollisionEnvironment>(new CollisionEnvironment(ik_module.robot_model) ) ;

  std::cout << "Test1\n";

  // Create Tasks
  std::shared_ptr<Task> pelvis_task(new Task6DPose(ik_module.robot_model, "pelvis"));
  std::shared_ptr<Task> lfoot_task(new Task6DPose(ik_module.robot_model, "leftCOP_Frame"));
  std::shared_ptr<Task> rfoot_task(new Task6DPose(ik_module.robot_model, "rightCOP_Frame"));
  std::shared_ptr<Task> rhand_task(new TaskSelfCollision(ik_module.robot_model, "rightPalm", collision, "rhand"));
  std::shared_ptr<Task> rknee_task(new TaskSelfCollision(ik_module.robot_model, "rightKneePitch", collision, "rknee"));

  std::cout << "Test2\n";

  // Stack Tasks in order of priority
  std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {pelvis_task, lfoot_task, rfoot_task, rhand_task, rknee_task}));

  // Set desired Pelvis configuration
  Eigen::Vector3d pelvis_des_pos;
  Eigen::Quaternion<double> pelvis_des_quat;  

   // Set desired Foot configuration
  Eigen::Vector3d rfoot_des_pos;
  Eigen::Quaternion<double> rfoot_des_quat;

  Eigen::Vector3d lfoot_des_pos;
  Eigen::Quaternion<double> lfoot_des_quat;

  // Pelvis should be 1m above the ground and the orientation must be identity
  pelvis_des_pos.setZero();
  pelvis_des_pos[2] = 1.05;
  pelvis_des_quat.setIdentity();

  // Foot should be flat on the ground and spaced out by 0.25m
  rfoot_des_pos.setZero();
  rfoot_des_pos[0] = 0.125;
  rfoot_des_pos[1] = -0.125;
  rfoot_des_quat.setIdentity();

  lfoot_des_pos.setZero();
  lfoot_des_pos[1] = 0.125;
  lfoot_des_quat.setIdentity();

  // Set Tsak references
  pelvis_task->setReference(pelvis_des_pos, pelvis_des_quat);
  lfoot_task->setReference(lfoot_des_pos, lfoot_des_quat);
  rfoot_task->setReference(rfoot_des_pos, rfoot_des_quat);

  std::cout << "Test3\n";

  // Get Errors -----------------------------------------------------------------------------
  Eigen::VectorXd task_error;
  rhand_task->getError(task_error);
  std::cout << "Right Hand Task Error = " << task_error.transpose() << std::endl;

  rknee_task->getError(task_error);
  std::cout << "Right Knee Task Error = " << task_error.transpose() << std::endl;

  ik_module.addTasktoHierarchy(task_stack_priority_1);

  int solve_result;
  double error_norm;
  Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(ik_module.robot_model->getDimQdot());

  ik_module.prepareNewIKDataStrcutures();
  ik_module.solveIK(solve_result, error_norm, q_sol);

  Eigen::VectorXd q_config = Eigen::VectorXd::Zero(q_init.size());
  printVec("q_sol", q_sol);

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
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  std::cout << "Initialize Valkyrie Model" << std::endl;
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