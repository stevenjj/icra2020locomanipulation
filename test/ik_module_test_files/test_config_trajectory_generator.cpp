#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>
#include "pinocchio/utils/timer.hpp"


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
  q_start[valkyrie->getJointIndex("rightAnklePitch")] = 0.0;//-0.3;

  q_start[valkyrie->getJointIndex("rightShoulderPitch")] = 0.2;
  q_start[valkyrie->getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie->getJointIndex("rightElbowPitch")] = 1.0 ; //0.4;
  q_start[valkyrie->getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie->getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}

void test_hand_in_place_config_trajectory_generator(){
  std::cout << "[Running Config Trajectory Generator Test] In place walking with hand constant" << std::endl;

  std::cout << "[ConfigTrajectoryGenerator] Constructed" << std::endl;
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  int N_resolution = 60; //single step = 30. two steps = 60// 30* number of footsteps
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
  footstep_2.position[0] -= 0.1;
  footstep_1.position[0] -= 0.1;
  std::vector<Footstep> input_footstep_list = {footstep_2, footstep_1};
  // std::vector<Footstep> input_footstep_list = {footstep_1};


  // Get current hand pose
  Eigen::Vector3d rhand_pos;
  Eigen::Quaterniond rhand_ori;
  valkyrie_model->getFrameWorldPose("rightPalm", rhand_pos, rhand_ori);  
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
  ctg.computeConfigurationTrajectory(q_start, input_footstep_list);
  std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  // Visualize Trajectory
  visualizer.visualizeConfigurationTrajectory(q_start, ctg.traj_q_config);

}

void test_initial_hand_location_stance(){
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  Eigen::VectorXd q_start, q_end;
  initialize_config(q_start, valkyrie_model);


  // Initialize the manipulation function for manipulating the door
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_trajectory.yaml";
  ManipulationFunction manipulate_door(door_yaml_file);
  // Set hinge location as waypoints are with respect to the hinge
  Eigen::Vector3d hinge_position(1.5, -0.5, 1.125);
  Eigen::Quaterniond hinge_orientation(0.707, 0, 0, 0.707);
  hinge_orientation.normalize();
  manipulate_door.setWorldTransform(hinge_position, hinge_orientation);

  // q_start[0] = 0.5;
  q_start[1] = 0.5;
  // q_start[valkyrie_model->getJointIndex("rightShoulderPitch")] = 0.0;
  // q_start[valkyrie_model->getJointIndex("rightShoulderRoll")] = 1.5;
  // q_start[valkyrie_model->getJointIndex("rightElbowPitch")] = 1.5 ; //0.4;
  // q_start[valkyrie_model->getJointIndex("rightForearmYaw")] = 1.0;

  // Get the starting pose of the hand 
  Eigen::Vector3d rpalm_des_pos;
  Eigen::Quaterniond rpalm_des_quat;
  double s_init = 0.0;
  manipulate_door.getPose(s_init, rpalm_des_pos, rpalm_des_quat);

  // Initialize the stance generation object
  ValkyrieStanceGeneration stance_generator(valkyrie_model);
  // Set the starting configuration
  stance_generator.setStartingConfig(q_start);
  // Set desired right hand and enable right hand use
  stance_generator.setUseRightHand(true);
  stance_generator.setDesiredRightHandPose(rpalm_des_pos, rpalm_des_quat);

  // Compute starting stance for the door opening task
  bool convergence = stance_generator.computeStance(q_end);
  stance_generator.snap_to_floor_ik_module.printSolutionResults();
  stance_generator.stance_ik_module.printSolutionResults();
  std::cout << "Stance Generation Result " << (convergence ? "true": "false") << std::endl;

  // Store the data
  YAML::Emitter out;
  out << YAML::BeginMap;
    data_saver::emit_joint_configuration(out, "robot_starting_configuration", q_end);
    data_saver::emit_position(out, "hinge_position", hinge_position);
    data_saver::emit_orientation(out, "hinge_orientation", hinge_orientation);
  out << YAML::EndMap;  
  std::cout << "Here's the output YAML:\n" << out.c_str() << "\n" << std::endl;
  std::ofstream file_output_stream("robot_door_initial_configuration.yaml");
  file_output_stream << out.c_str();
  // End of Storing the data


  // Create visualization object
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(ros_node, valkyrie_model);  
  visualizer.visualizeConfiguration(q_start, q_end);


}

void load_robot_door_configuration(Eigen::VectorXd & q_out, Eigen::Vector3d & hinge_pos_out, Eigen::Quaterniond & hinge_ori_out){
  ParamHandler param_handler;
  param_handler.load_yaml_file(THIS_PACKAGE_PATH"stored_configurations/robot_door_initial_configuration.yaml");  

  // Get the robot configuration vector
  std::vector<double> robot_q;
  param_handler.getVector("robot_starting_configuration", robot_q);
  
  // Get hinge location and orientation
  double hinge_x, hinge_y, hinge_z, hinge_rx, hinge_ry, hinge_rz, hinge_rw;
  param_handler.getNestedValue({"hinge_position", "x"}, hinge_x);
  param_handler.getNestedValue({"hinge_position", "y"}, hinge_y);
  param_handler.getNestedValue({"hinge_position", "z"}, hinge_z);
  param_handler.getNestedValue({"hinge_orientation", "x"}, hinge_rx);
  param_handler.getNestedValue({"hinge_orientation", "y"}, hinge_ry);
  param_handler.getNestedValue({"hinge_orientation", "z"}, hinge_rz);
  param_handler.getNestedValue({"hinge_orientation", "w"}, hinge_rw);

  // Convert to Eigen data types
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_q.size());
  for(int i = 0; i < robot_q.size(); i++){
    q[i] = robot_q[i];
  }
  Eigen::Vector3d hinge_position(hinge_x, hinge_y, hinge_z);
  Eigen::Quaterniond hinge_orientation(hinge_rw, hinge_rx, hinge_ry, hinge_rz);

  // Set outputs
  q_out = q;
  hinge_pos_out = hinge_position;
  hinge_ori_out = hinge_orientation;
}


void test_door_open_config_trajectory(){
  // Initialize the robot
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  Eigen::VectorXd q_start, q_start_door;
  initialize_config(q_start, valkyrie_model);

  // Get the Initial Robot and Door Configuration
  Eigen::Vector3d hinge_position;
  Eigen::Quaterniond hinge_orientation;
  load_robot_door_configuration(q_start_door, hinge_position, hinge_orientation);

  // Initialize the manipulation function for manipulating the door
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_trajectory.yaml";
  std::shared_ptr<ManipulationFunction> f_s_manipulate_door(new ManipulationFunction(door_yaml_file));
  // Set hinge location as waypoints are with respect to the hinge
  hinge_orientation.normalize();
  f_s_manipulate_door->setWorldTransform(hinge_position, hinge_orientation);

  // Visualize the loaded configuration
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(ros_node, valkyrie_model);  
  // visualizer.visualizeConfiguration(q_start, q_start_door);

  // Attempt to take a footstep with this configuration
  // Create the config trajectory object
  int N_resolution = 60; //single step = 30. two steps = 60// 30* number of footsteps
  ConfigTrajectoryGenerator ctg(valkyrie_model, N_resolution);
  valkyrie_model->updateFullKinematics(q_start_door);

  // Create footsteps
  Footstep footstep_1; footstep_1.setLeftSide();
  Footstep footstep_2; footstep_2.setRightSide();

  valkyrie_model->getFrameWorldPose("leftCOP_Frame", footstep_1.position, footstep_1.orientation);  
  valkyrie_model->getFrameWorldPose("rightCOP_Frame", footstep_2.position, footstep_2.orientation);  

  double theta = 20.0/180.0*M_PI;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Quaternion<double> quat(aa);

  footstep_1.position[0] -= 0.1;

  footstep_2.position[0] -= 0.1;
  footstep_2.position[1] -= 0.1;

  footstep_2.orientation = quat;
  std::cout << "quat w = " << quat.w();
  footstep_2.R_ori = quat.toRotationMatrix(); 


  // Try no footsteps
  // std::vector<Footstep> input_footstep_list;
    // Try with footsteps
  std::vector<Footstep> input_footstep_list = {footstep_1, footstep_2};


  // Get current hand pose
  Eigen::Vector3d rhand_pos;
  Eigen::Quaterniond rhand_ori;
  valkyrie_model->getFrameWorldPose("rightPalm", rhand_pos, rhand_ori);  
  // Set Constant Right Hand trajectory to current //To do. set task gain for hand to be small in orientation
  ctg.setConstantRightHandTrajectory(rhand_pos, rhand_ori);
  ctg.setUseRightHand(true);
  ctg.setUseTorsoJointPosition(false);
  ctg.reinitializeTaskStack();

    // timer
  PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);

  // Have fast double support times
  ctg.wpg.setDoubleSupportTime(0.2);
  // Set Manipulation Only time to 3 seconds
  ctg.setManipulationOnlyTime(3.0);
  // Set Verbosity
  ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2);
  // Solve for configurations
  double s_o = 0.0;
  double delta_s = 0.15;

  timer.tic();
  ctg.computeConfigurationTrajectory(f_s_manipulate_door, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                 s_o, delta_s, q_start_door, input_footstep_list);


  std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;
  // Visualize Trajectory
  visualizer.visualizeConfigurationTrajectory(f_s_manipulate_door, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, q_start_door, ctg.traj_q_config);

}


void test_walking_config_trajectory_generator(){
  std::cout << "[Running Config Trajectory Generator Test]" << std::endl;

  std::cout << "[ConfigTrajectoryGenerator] Constructed" << std::endl;
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  int N_resolution = 100;
  ConfigTrajectoryGenerator ctg(valkyrie_model, N_resolution);

  Eigen::VectorXd q_start, q_end;
  initialize_config(q_start, valkyrie_model);

  // Test initial configuration computation for flat ground
  ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_1);
  ctg.computeInitialConfigForFlatGround(q_start, q_end);


  // Visualize the robot
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(ros_node, valkyrie_model);  
  // visualizer.visualizeConfiguration(q_start, q_end);

  // Update Initial
  q_start = q_end;
  valkyrie_model->updateFullKinematics(q_start);

  // Create footsteps
  Footstep footstep_1; footstep_1.setLeftSide();
  Footstep footstep_2; footstep_2.setRightSide();
  Footstep footstep_3; footstep_3.setLeftSide();
  Footstep footstep_4; footstep_4.setRightSide();
  valkyrie_model->getFrameWorldPose("leftCOP_Frame", footstep_1.position, footstep_1.orientation);  
  valkyrie_model->getFrameWorldPose("rightCOP_Frame", footstep_2.position, footstep_2.orientation);  

  footstep_3 = footstep_1;
  footstep_4 = footstep_2;

  // Walk forward 4 steps
  footstep_1.position[0] += 0.20;
  footstep_2.position[0] += 0.40;
  footstep_3.position[0] += 0.60;
  footstep_4.position[0] += 0.60;

  // Create Footstep list
  std::vector<Footstep> input_footstep_list = {footstep_1, footstep_2, footstep_3, footstep_4};

    // timer
  PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);

  // Have fast double support times
  ctg.wpg.setDoubleSupportTime(0.2);

  // Set Verbosity
  ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2);
  // Solve for configurations
  timer.tic();
  ctg.computeConfigurationTrajectory(q_start, input_footstep_list);
  std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  // Visualize Trajectory
  visualizer.visualizeConfigurationTrajectory(q_start, ctg.traj_q_config);

}

int main(int argc, char ** argv){   
  ros::init(argc, argv, "test_config_trajectory_generator");
  // test_walking_config_trajectory_generator();
  // test_hand_in_place_config_trajectory_generator();
  // test_initial_hand_location_stance();
  test_door_open_config_trajectory();

  return 0;
}