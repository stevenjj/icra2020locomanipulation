#include <iostream>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>

#include <avatar_locomanipulation/tasks/task.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>
#include <avatar_locomanipulation/tasks/task_objectcollision.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose_wrt_midfeet.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include "visualization_msgs/Marker.h"


void visualize_robot(Eigen::VectorXd & q_start, Eigen::VectorXd & q_end);

void printVec(const std::string & vec_name, const::Eigen::VectorXd vec){
  std::cout << vec_name << ": ";
  for(int i = 0; i < vec.size(); i++){
    std::cout << vec[i] << ", ";
  } 
  std::cout << std::endl;
}


std::shared_ptr<RobotModel> initialize_config(Eigen::VectorXd & q_init, Eigen::VectorXd & box_init){

	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

	// Initialize Valkyrie RobotModel
	std::shared_ptr<RobotModel> valkyrie(new RobotModel(filename, meshDir, srdf_filename) );

  filename = THIS_PACKAGE_PATH"models/simplebox2.urdf";
  meshDir  = THIS_PACKAGE_PATH"models/box/";

  // Initialize Boxes RobotModel
  std::shared_ptr<RobotModel> box1(new RobotModel(filename, meshDir) );


  // Define the configuration of Val
  Eigen::VectorXd q_start;
  q_start = Eigen::VectorXd::Zero(valkyrie->getDimQ());

  double theta = 0;//M_PI/4.0;
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

  q_start[valkyrie->getJointIndex("rightShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie->getJointIndex("rightElbowPitch")] = 0.4;
  q_start[valkyrie->getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie->getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;

  // Define the configuration of the cart
  Eigen::VectorXd q_box1;
  q_box1 = Eigen::VectorXd::Zero(box1->getDimQ());
  q_box1[0] = 0.33;  q_box1[1] = -0.524;  q_box1[2] = 0.813;//0.475 x
  // q_box1[0] = -0.06;  q_box1[1] = -0.0085;  q_box1[2] = -0.04;
  double theta1 = 0;//M_PI/4.0;	
  Eigen::AngleAxis<double> bb(theta1, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left	
  Eigen::Quaternion<double> quat_init; quat_init =  bb;
  q_box1[3] = quat_init.x();// 0.0;	
  q_box1[4] = quat_init.y(); //0.0;
  q_box1[5] = quat_init.z(); //sin(theta/2.0);
  q_box1[6] = quat_init.w(); //cos(theta/2.0);

  box_init = q_box1;

  return box1;
  
}

void testIK_module(){
  std::cout << "[IK Module Test]" << std::endl;
  Eigen::VectorXd q_init, box_init;
  std::shared_ptr<RobotModel> box1 = initialize_config(q_init, box_init);

  std::string filename, meshDir;
  filename = THIS_PACKAGE_PATH"models/simplebox1.urdf";
  meshDir  = THIS_PACKAGE_PATH"models/box/";

  // Initialize Boxes RobotModel
  std::shared_ptr<RobotModel> box2(new RobotModel(filename, meshDir) );

   // Create IK Module
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir_  = THIS_PACKAGE_PATH"../val_model/"; 
  std::cout << "Initialize Valkyrie Model" << std::endl;
  std::shared_ptr<RobotModel> valkyrie(new RobotModel(urdf_filename, meshDir_, srdf_filename));
  
  IKModule ik_module(valkyrie);
  ik_module.setInitialConfig(q_init);  

  // Update Robot Kinematics
  ik_module.robot_model->enableUpdateGeomOnKinematicsUpdate(true);
  ik_module.robot_model->updateFullKinematics(q_init);
  box1->enableUpdateGeomOnKinematicsUpdate(true);
  box1->updateFullKinematics(box_init);

  Eigen::Vector3d cur_pos;
  Eigen::Quaternion<double> cur_ori;
  ik_module.robot_model->getFrameWorldPose("rightPalm", cur_pos, cur_ori);
  std::cout << "Rhand cur_pos\n" << cur_pos << std::endl;


  std::shared_ptr<CollisionEnvironment> collision(new CollisionEnvironment(ik_module.robot_model) );

  std::string prefix = "box1";
  collision->add_new_object(box1, box_init, prefix);

  prefix = "box2";
  box_init[0] = 0.83;
  box_init[1] = 0.524;
  box2->enableUpdateGeomOnKinematicsUpdate(true);
  box2->updateFullKinematics(box_init);
  collision->add_new_object(box2, box_init, prefix);

  // Create Tasks
  std::shared_ptr<Task> pelvis_task(new Task6DPose(ik_module.robot_model, "pelvis"));
  std::shared_ptr<Task> lfoot_task(new Task6DPose(ik_module.robot_model, "leftCOP_Frame"));
  std::shared_ptr<Task> rfoot_task(new Task6DPose(ik_module.robot_model, "rightCOP_Frame"));

  std::shared_ptr<Task> rhand_task(new TaskObjectCollision(ik_module.robot_model, "rightPalm", collision, "rhand"));
  std::shared_ptr<Task> lhand_task(new TaskObjectCollision(ik_module.robot_model, "leftPalm", collision, "lhand"));
  // std::shared_ptr<Task> pelvis_object_task(new TaskObjectCollision(ik_module.robot_model, "pelvis", collision, "pelvis"));

  rhand_task->setTaskGain(0.1);
  lhand_task->setTaskGain(0.1);
  // pelvis_object_task->setTaskGain(20.0);


  // Stack Tasks in order of priority
  std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {rhand_task, lhand_task, lfoot_task, rfoot_task}));

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
  rfoot_des_pos[0] = 0.0; //0.125;
  rfoot_des_pos[1] = -0.125;
  rfoot_des_quat.setIdentity();

  lfoot_des_pos.setZero();
  lfoot_des_pos[1] = 0.125;
  lfoot_des_quat.setIdentity();

  // Set Tsak references
  pelvis_task->setReference(pelvis_des_pos, pelvis_des_quat);
  lfoot_task->setReference(lfoot_des_pos, lfoot_des_quat);
  rfoot_task->setReference(rfoot_des_pos, rfoot_des_quat);

  // Get Errors -----------------------------------------------------------------------------
  Eigen::VectorXd task_error;
  rhand_task->getError(task_error);
  std::cout << "Right Hand Task Error = " << task_error.transpose() << std::endl;
  lhand_task->getError(task_error);
  std::cout << "Left Hand Task Error = " << task_error.transpose() << std::endl;
  // pelvis_object_task->getError(task_error);
  // std::cout << "Pelvis Object Task Error = " << task_error.transpose() << std::endl;

  ik_module.addTasktoHierarchy(task_stack_priority_1);

  int solve_result;
  double error_norm;
  Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(ik_module.robot_model->getDimQdot());

  double error_tol = 1e-6;
  ik_module.setErrorTol(error_tol);

  ik_module.prepareNewIKDataStrcutures();
  ik_module.solveIK(solve_result, error_norm, q_sol);
  ik_module.printSolutionResults();

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

  // Visualize the two boxes
  ros::Publisher point1_pub = n.advertise<visualization_msgs::Marker>("point1", 100);
  ros::Publisher point2_pub = n.advertise<visualization_msgs::Marker>("point2", 100);

  visualization_msgs::Marker point1, point2;

  // Fill the Points marker msg
  point1.pose.position.x = 0.33;
  point1.pose.position.y = -0.524;
  point1.pose.position.z = 0.813;
  point1.pose.orientation.y = 0.;
  point1.pose.orientation.z = 0.;
  point1.pose.orientation.w = 1.;
  point1.scale.x = 0.02;
  point1.scale.y = 0.02;
  point1.scale.z = 0.02;
  point1.header.frame_id = "world";
  point1.text = "point1";
  point1.color.r = 0.0f;
  point1.color.g = 1.0f;
  point1.color.b = 0.0f;
  point1.color.a = 1.0;
  point1.action = visualization_msgs::Marker::ADD;
  point1.type = visualization_msgs::Marker::CUBE;

  // Fill the Points marker msg
  point2.pose.position.x = 0.83;
  point2.pose.position.y = 0.524;
  point2.pose.position.z = 0.813;
  point2.pose.orientation.y = 0.;
  point2.pose.orientation.z = 0.;
  point2.pose.orientation.w = 1.;
  point2.scale.x = 0.01;
  point2.scale.y = 0.01;
  point2.scale.z = 0.01;
  point2.header.frame_id = "world";
  point2.text = "point2";
  point2.color.r = 0.0f;
  point2.color.g = 1.0f;
  point2.color.b = 0.0f;
  point2.color.a = 1.0;
  point2.action = visualization_msgs::Marker::ADD;
  point2.type = visualization_msgs::Marker::CUBE;

  while (ros::ok()){
      br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
      robot_joint_state_pub.publish(joint_msg_init);

      br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
      robot_ik_joint_state_pub.publish(joint_msg_end);

      point1_pub.publish(point1);
      point2_pub.publish(point2);
    ros::spinOnce();
    loop_rate.sleep();
  }
}




int main(int argc, char ** argv){
  ros::init(argc, argv, "test_ik_module");
  testIK_module();
  return 0;
}