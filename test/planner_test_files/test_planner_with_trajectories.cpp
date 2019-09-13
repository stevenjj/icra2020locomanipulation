#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>
#include "pinocchio/utils/timer.hpp"

// Planner
#include <avatar_locomanipulation/planners/a_star_planner.hpp>
#include <avatar_locomanipulation/planners/locomanipulation_a_star_planner.hpp>

// CPP version of the neural network
#include <avatar_locomanipulation/helpers/NeuralNetModel.hpp>
#include <avatar_locomanipulation/helpers/IOUtilities.hpp>

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


// Timer
#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

// Enabling multi-threading
#include <omp.h> 

using namespace planner;


void load_initial_robot_door_configuration(Eigen::VectorXd & q_out, Eigen::Vector3d & hinge_pos_out, Eigen::Quaterniond & hinge_ori_out){
  ParamHandler param_handler;
  param_handler.load_yaml_file(THIS_PACKAGE_PATH"stored_configurations/robot_door_initial_configuration_v3.yaml");  

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


void load_final_robot_door_configuration(Eigen::VectorXd & q_out){
  ParamHandler param_handler;
  param_handler.load_yaml_file(THIS_PACKAGE_PATH"stored_configurations/robot_door_final_configuration.yaml");  

  // Get the robot configuration vector
  std::vector<double> robot_q;
  param_handler.getVector("robot_configuration", robot_q);

  // Convert to Eigen data types
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_q.size());
  for(int i = 0; i < robot_q.size(); i++){
    q[i] = robot_q[i];
  }
  // Set outputs
  q_out = q;
}


void test_door_open_config_trajectory(){
  // Initialize the robot
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  // Get the Initial Robot and Door Configuration
  Eigen::VectorXd q_start_door;
  Eigen::Vector3d hinge_position;
  Eigen::Quaterniond hinge_orientation;
  load_initial_robot_door_configuration(q_start_door, hinge_position, hinge_orientation);

  // Initialize the manipulation function for manipulating the door
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_open_trajectory_v3.yaml";
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
  footstep_1.position[0] -= 0.1;

  footstep_2.position[0] -= 0.1;
  footstep_2.position[1] -= 0.1;

  // Try no footsteps
  // std::vector<Footstep> input_footstep_list;
    // Try with footsteps
  std::vector<Footstep> input_footstep_list = {footstep_1, footstep_2};


  // Initialize Trajectory Generation Module
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
  bool convergence = ctg.computeConfigurationTrajectory(f_s_manipulate_door, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                     s_o, delta_s, q_start_door, input_footstep_list);

  std::cout << "Converged? " << (convergence ? "True" : "False") << std::endl; 

  std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;
  // Visualize Trajectory
  visualizer.visualizeConfigurationTrajectory(q_start_door, ctg.traj_q_config);

}

void test_final_configuration(){
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  // Get the Initial Robot and Door Configuration
  Eigen::VectorXd q_start_door;
  Eigen::Vector3d hinge_position;
  Eigen::Quaterniond hinge_orientation;
  load_initial_robot_door_configuration(q_start_door, hinge_position, hinge_orientation);

  // Update the model
  valkyrie_model->updateFullKinematics(q_start_door);

  // Initialize the manipulation function for manipulating the door
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_open_trajectory_v3.yaml";
  std::shared_ptr<ManipulationFunction> f_s_manipulate_door(new ManipulationFunction(door_yaml_file));
  // Set hinge location as waypoints are with respect to the hinge
  hinge_orientation.normalize();
  f_s_manipulate_door->setWorldTransform(hinge_position, hinge_orientation);

  // Get the starting pose of the hand 
  Eigen::Vector3d rpalm_init_pos;
  Eigen::Quaterniond rpalm_init_quat;
  double s_init = 0.0;
  f_s_manipulate_door->getPose(s_init, rpalm_init_pos, rpalm_init_quat);

  // Get the ending pose 
  Eigen::Vector3d rpalm_final_pos;
  Eigen::Quaterniond rpalm_final_quat;
  double s_final = 0.999;
  f_s_manipulate_door->getPose(s_final, rpalm_final_pos, rpalm_final_quat);


  // q_end of the robot
  Eigen::VectorXd q_end_door = q_start_door;

  // Get initial pelvis position and quaternion
  Eigen::Vector3d pelvis_init_pos = q_start_door.head(3);
  Eigen::Quaterniond pelvis_init_quat(q_start_door[6], q_start_door[3], q_start_door[4], q_start_door[5]);

  std::cout << "pelvis pos = "  << pelvis_init_pos.transpose() << std::endl;
  std::cout << "pelvis quat = "  << pelvis_init_quat.x() << ", " << pelvis_init_quat.y() << ", " << pelvis_init_quat.z() << ", " << pelvis_init_quat.w() << std::endl;

  // Get the hand to pelvis transform:
  Eigen::Matrix3d R_hand_ori_T = rpalm_init_quat.toRotationMatrix().transpose();
  Eigen::Vector3d pelvis_pos_rhand_frame = R_hand_ori_T*(pelvis_init_pos - rpalm_init_pos);
  Eigen::Quaterniond pelvis_quat_rhand_frame = rpalm_init_quat.inverse()*pelvis_init_quat; 

  Eigen::Vector3d pelvis_pos_again = rpalm_init_quat.toRotationMatrix()*pelvis_pos_rhand_frame + rpalm_init_pos; 
  Eigen::Quaterniond pelvis_ori_again = rpalm_init_quat*pelvis_quat_rhand_frame;

  std::cout << "pelvis pos again = "  << pelvis_pos_again.transpose() << std::endl;
  std::cout << "pelvis quat again = "  << pelvis_ori_again.x() << ", " << pelvis_ori_again.y() << ", " << pelvis_ori_again.z() << ", " << pelvis_ori_again.w() << std::endl;

  // compute end configuration of the pelvis
  Eigen::Quaternion<double> end_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  end_quat = rpalm_final_quat*pelvis_quat_rhand_frame;
  q_end_door.head(3) = rpalm_final_quat.toRotationMatrix()*pelvis_pos_rhand_frame + rpalm_final_pos;

  q_end_door[3] = end_quat.x();
  q_end_door[4] = end_quat.y();
  q_end_door[5] = end_quat.z();
  q_end_door[6] = end_quat.w();

  // Store the data
  // YAML::Emitter out;
  // out << YAML::BeginMap;
  //   data_saver::emit_joint_configuration(out, "robot_configuration", q_end_door);
  // out << YAML::EndMap;  
  // std::cout << "Here's the output YAML:\n" << out.c_str() << "\n" << std::endl;
  // std::ofstream file_output_stream("robot_door_final_configuration.yaml");
  // file_output_stream << out.c_str();
  // End of Storing the data

  // Eigen::VectorXd q_final_door;
  // load_final_robot_door_configuration(q_final_door);

  // Visualize starting and ending configurations:
  // Create visualization object
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(ros_node, valkyrie_model);  
  visualizer.visualizeConfiguration(q_start_door, q_end_door);

}


void test_LM_planner(){
  // Initialize the robot ---------------------------------------------------------------------------------------
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));
  // Get the Initial Robot and Door Configuration
  Eigen::VectorXd q_start_door;
  Eigen::Vector3d hinge_position;
  Eigen::Quaterniond hinge_orientation;
  load_initial_robot_door_configuration(q_start_door, hinge_position, hinge_orientation);

  // Get the final guessed robot configuration
  Eigen::VectorXd q_final_door;
  load_final_robot_door_configuration(q_final_door);

  // Initialize the manipulation function for manipulating the door ----------------------------------
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_open_trajectory_v3.yaml";
  std::shared_ptr<ManipulationFunction> f_s_manipulate_door(new ManipulationFunction(door_yaml_file));
  // Set hinge location as waypoints are with respect to the hinge
  hinge_orientation.normalize();
  f_s_manipulate_door->setWorldTransform(hinge_position, hinge_orientation);

  // Initialize Config Trajectory Generator
  int N_resolution = 60; //single step = 30. two steps = 60// 30* number of footsteps
  std::shared_ptr<ConfigTrajectoryGenerator> ctg(new ConfigTrajectoryGenerator(valkyrie_model, N_resolution));
  valkyrie_model->updateFullKinematics(q_start_door);

  // Initialize Trajectory Generation Module
  ctg->setUseRightHand(true);
  ctg->setUseTorsoJointPosition(true);
  // ctg->setUseTorsoJointPosition(false);
  ctg->reinitializeTaskStack();
  // timer
  //PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);

  // Set Double support times
  ctg->wpg.setDoubleSupportTime(0.2); // Sets the desired double support time
  // ctg->wpg.setSettlingPercentage(0.01); // Percentage to settle. Default 0.999
  ctg->wpg.setSettlingPercentage(0.99); // Percentage to settle. Default 0.999
  // Set Manipulation Only time to 3 seconds
  ctg->setManipulationOnlyTime(3.0);
  // Set Verbosity
  ctg->setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_0);

  // End of initialization ---------------------------------------------------------------------------------------

  LocomanipulationPlanner lm_planner;
  // Initialize
  lm_planner.initializeLocomanipulationVariables(valkyrie_model, f_s_manipulate_door, ctg);

  // 
  double s_init = 0.0;
  double s_goal = 0.99;//0.99; //0.20; //0.12;//0.08;
  shared_ptr<Node> starting_vertex (std::make_shared<LMVertex>(s_init, q_start_door));    
  shared_ptr<Node> goal_vertex (std::make_shared<LMVertex>(s_goal, q_final_door));

  lm_planner.setStartNode(starting_vertex);
  lm_planner.setGoalNode(goal_vertex);
  
   // Test Planner speed:
  auto t1 = Clock::now();
  auto t2 = Clock::now();
  double time_span;

  t1 = Clock::now();
  bool a_star_success = lm_planner.doAstar();
  t2 = Clock::now();

  time_span = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1).count();
  std::cout << " planner time = " << time_span << " seconds" << std::endl;

  if (a_star_success){
    // Visualize the trajectory:
    // std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
    // RVizVisualizer visualizer(ros_node, valkyrie_model);  
    // visualizer.visualizeConfigurationTrajectory(q_start_door, lm_planner.path_traj_q_config);

    // Construct the path
    std::cout << "Constructing the dynamic trajectory" << std::endl;
    std::cout << "Path has size: " << lm_planner.optimal_path.size() << std::endl;
    if (lm_planner.reconstructConfigurationTrajectoryv2()){
      // Visualize the trajectory:
      std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
      RVizVisualizer visualizer(ros_node, valkyrie_model);  
      visualizer.visualizeConfigurationTrajectory(q_start_door, lm_planner.path_traj_q_config);      
    }else{
      std::cout << "non convergence" << std::endl;
    }


  }


}


void test_LM_planner_with_NN(){
  // Service name for neural network
  std::string service_name = "locomanipulation_feasibility_classifier";

  // Create ROS handle and client
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  ros::ServiceClient client = ros_node->serviceClient<avatar_locomanipulation::BinaryClassifierQuery>(service_name);

  // Wait for the service
  std::cout << "waiting for " << service_name << " service..." << std::endl;
  ros::service::waitForService(service_name);
  std::cout << "service available" << std::endl;


  // Initialize the robot ---------------------------------------------------------------------------------------
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));
  // Get the Initial Robot and Door Configuration
  Eigen::VectorXd q_start_door;
  Eigen::Vector3d hinge_position;
  Eigen::Quaterniond hinge_orientation;
  load_initial_robot_door_configuration(q_start_door, hinge_position, hinge_orientation);

  // Get the final guessed robot configuration
  Eigen::VectorXd q_final_door;
  load_final_robot_door_configuration(q_final_door);

  // Initialize the manipulation function for manipulating the door ----------------------------------
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_open_trajectory_v3.yaml";
  std::shared_ptr<ManipulationFunction> f_s_manipulate_door(new ManipulationFunction(door_yaml_file));
  // Set hinge location as waypoints are with respect to the hinge
  hinge_orientation.normalize();
  f_s_manipulate_door->setWorldTransform(hinge_position, hinge_orientation);

  // Initialize Config Trajectory Generator
  int N_resolution = 60; //single step = 30. two steps = 60// 30* number of footsteps
  std::shared_ptr<ConfigTrajectoryGenerator> ctg(new ConfigTrajectoryGenerator(valkyrie_model, N_resolution));
  valkyrie_model->updateFullKinematics(q_start_door);

  // Initialize Trajectory Generation Module
  ctg->setUseRightHand(true);
  // ctg->setUseTorsoJointPosition(false);
  ctg->setUseTorsoJointPosition(true);
  ctg->reinitializeTaskStack();
  // timer
  //PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);

  // Have fast double support times
  ctg->wpg.setDoubleSupportTime(0.2);
  // Set Manipulation Only time to 3 seconds
  ctg->setManipulationOnlyTime(3.0);
  // Set Verbosity
  ctg->setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_0);

  // End of initialization ---------------------------------------------------------------------------------------
  LocomanipulationPlanner lm_planner;

  // Set whether to learn from mistakes or not
  // Regenerate the discretizations
  bool learn_from_mistakes = false;
  if (learn_from_mistakes){ 
    lm_planner.classifier_store_mistakes = true;  
    std::cout << "Storing classifier mistakes? " << lm_planner.classifier_store_mistakes << std::endl;
    lm_planner.generateDiscretization();
  }
  // ----------

  // Set to trust the classifier
  lm_planner.trust_classifier = true; //false;

  // Initialize
  lm_planner.initializeLocomanipulationVariables(valkyrie_model, f_s_manipulate_door, ctg);
  // Set the classifier client:
  std::cout << "Setting the classifier client" << std::endl;
  lm_planner.setClassifierClient(client);

  double s_init = 0.0;
  double s_goal = 0.99; //0.32; //0.16; //0.20; //0.12;//0.08;
  shared_ptr<Node> starting_vertex (std::make_shared<LMVertex>(s_init, q_start_door));    
  shared_ptr<Node> goal_vertex (std::make_shared<LMVertex>(s_goal, q_final_door));

  lm_planner.setStartNode(starting_vertex);
  lm_planner.setGoalNode(goal_vertex);
  
  bool a_star_success = lm_planner.doAstar();
  if (a_star_success){
    // Construct the path
    // std::cout << "Constructing the path" << std::endl;
    // lm_planner.constructPath();
    // std::cout << "Path has size: " << lm_planner.optimal_path.size() << std::endl;
    // std::cout << "reconstructing the total trajectory" << std::endl;
    // lm_planner.reconstructConfigurationTrajectory();

    // Visualize the trajectory:
    RVizVisualizer visualizer(ros_node, valkyrie_model);  
    visualizer.visualizeConfigurationTrajectory(q_start_door, lm_planner.path_traj_q_config);

  }  
}


void test_LM_planner_with_cpp_NN(){
  // // Create ROS handle and client
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());

  // // Service name for neural network
  // std::string service_name = "locomanipulation_feasibility_classifier";
  // ros::ServiceClient client = ros_node->serviceClient<avatar_locomanipulation::BinaryClassifierQuery>(service_name);

  // // Wait for the service
  // std::cout << "waiting for " << service_name << " service..." << std::endl;
  // ros::service::waitForService(service_name);
  // std::cout << "service available" << std::endl;

  // Initialize the neural network --------------------------------------------------------------
  ParamHandler param_handler;
  std::string model_path = THIS_PACKAGE_PATH"nn_models/layer3_20000pts/cpp_model/layer3_model.yaml";

  std::cout << "Loading Model..." << std::endl;
  myYAML::Node model = myYAML::LoadFile(model_path);
  std::shared_ptr<NeuralNetModel> nn_transition_model(new NeuralNetModel(model, false));

  std::cout << "Loaded" << std::endl;

  // Load neural network normalization Params
  param_handler.load_yaml_file(THIS_PACKAGE_PATH"nn_models/layer3_20000pts/cpp_model/normalization_params.yaml");
  std::vector<double> vmean;
  param_handler.getVector("x_train_mean", vmean);
  std::vector<double> vstd_dev;
  param_handler.getVector("x_train_std", vstd_dev);

  Eigen::VectorXd mean(32);
  Eigen::VectorXd std_dev(32);
  for (int ii = 0; ii < 32; ii++){
    mean[ii] = vmean[ii];
    std_dev[ii] = vstd_dev[ii];
  } 

  
  // Initialize the robot ---------------------------------------------------------------------------------------
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));
  // Get the Initial Robot and Door Configuration
  Eigen::VectorXd q_start_door;
  Eigen::Vector3d hinge_position;
  Eigen::Quaterniond hinge_orientation;
  load_initial_robot_door_configuration(q_start_door, hinge_position, hinge_orientation);

  // Get the final guessed robot configuration
  Eigen::VectorXd q_final_door;
  load_final_robot_door_configuration(q_final_door);

  // Initialize the manipulation function for manipulating the door ----------------------------------
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_open_trajectory_v3.yaml";
  std::shared_ptr<ManipulationFunction> f_s_manipulate_door(new ManipulationFunction(door_yaml_file));
  // Set hinge location as waypoints are with respect to the hinge
  hinge_orientation.normalize();
  f_s_manipulate_door->setWorldTransform(hinge_position, hinge_orientation);

  // Initialize Config Trajectory Generator
  int N_resolution = 60; //single step = 30. two steps = 60// 30* number of footsteps
  std::shared_ptr<ConfigTrajectoryGenerator> ctg(new ConfigTrajectoryGenerator(valkyrie_model, N_resolution));
  valkyrie_model->updateFullKinematics(q_start_door);

  // Initialize Trajectory Generation Module
  ctg->setUseRightHand(true);
  // ctg->setUseTorsoJointPosition(false);
  ctg->setUseTorsoJointPosition(true);
  ctg->reinitializeTaskStack();
  // timer
  //PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);

  // Have fast double support times
  ctg->wpg.setDoubleSupportTime(0.2);
  // Set Manipulation Only time to 3 seconds
  ctg->setManipulationOnlyTime(3.0);
  // Set Verbosity
  ctg->setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_0);

  // End of initialization ---------------------------------------------------------------------------------------
  LocomanipulationPlanner lm_planner;

  // Set whether to learn from mistakes or not
  // Regenerate the discretizations
  bool learn_from_mistakes = false;
  if (learn_from_mistakes){ 
    lm_planner.classifier_store_mistakes = true;  
    std::cout << "Storing classifier mistakes? " << lm_planner.classifier_store_mistakes << std::endl;
    lm_planner.generateDiscretization();
  }
  // ----------

  // Set to trust the classifier
  lm_planner.trust_classifier = true; //false;

  // Initialize
  lm_planner.initializeLocomanipulationVariables(valkyrie_model, f_s_manipulate_door, ctg);
  // Set the nn classifier
  lm_planner.setNeuralNetwork(nn_transition_model, mean, std_dev);

  double s_init = 0.0;
  double s_goal = 0.99; //0.32; //0.16; //0.20; //0.12;//0.08;
  shared_ptr<Node> starting_vertex (std::make_shared<LMVertex>(s_init, q_start_door));    
  shared_ptr<Node> goal_vertex (std::make_shared<LMVertex>(s_goal, q_final_door));

  lm_planner.setStartNode(starting_vertex);
  lm_planner.setGoalNode(goal_vertex);
 
   // Test Planner speed:
  auto t1 = Clock::now();
  auto t2 = Clock::now();
  double time_span;

  t1 = Clock::now();
  bool a_star_success = lm_planner.doAstar();
  t2 = Clock::now();

  time_span = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1).count();
  std::cout << " planner time = " << time_span << " seconds" << std::endl;

  if (a_star_success){
    // Construct the path
    // std::cout << "Constructing the path" << std::endl;
    // lm_planner.constructPath();
    // std::cout << "Path has size: " << lm_planner.optimal_path.size() << std::endl;
    // std::cout << "reconstructing the total trajectory" << std::endl;
    // lm_planner.reconstructConfigurationTrajectory();

    // Visualize the trajectory:
    RVizVisualizer visualizer(ros_node, valkyrie_model);  
    visualizer.visualizeConfigurationTrajectory(q_start_door, lm_planner.path_traj_q_config);

  }  
}


void test_planner(){
    FootstepPlanner footstepplanner;

    double lfx_i = -1.0;
    double lfy_i = 1.0;
    double rfx_i = -1.0;
    double rfy_i = -1.0;

    shared_ptr<Node> begin_footstep (std::make_shared<FootstepNode>(lfx_i ,lfy_i, rfx_i, rfy_i, 0.0, 0.0,"RF",0.0));
    double lfx_g = 15.0;
    double lfy_g = 9.0;
    double rfx_g = 15.0;
    double rfy_g = 7.0;
    // shared_ptr<Node> goal_footstep (std::make_shared<FootstepNode>(lfx_g, lfy_g, rfx_g, rfy_g, -M_PI/4.0, -M_PI/4.0,false));
    shared_ptr<Node> goal_footstep (std::make_shared<FootstepNode>(lfx_g, lfy_g, rfx_g, rfy_g, M_PI/4, M_PI/4,"RF",0.0));

    footstepplanner.setStartNode(begin_footstep);
    footstepplanner.setGoalNode(goal_footstep);

    footstepplanner.doAstar();

    footstepplanner.printPath();
}

int main(int argc, char ** argv){   
  ros::init(argc, argv, "test_planner_with_trajectories");

  // Enabling multi-threading
  int num_cores = 4;
  omp_set_num_threads(num_cores);
  Eigen::setNbThreads(num_cores);
  Eigen::initParallel();
  int num_threads = Eigen::nbThreads();
  std::cout<<"num_threads: "<<num_threads<<std::endl;

  // test_final_configuration();
  test_LM_planner();
  // test_LM_planner_with_NN();
  // test_LM_planner_with_cpp_NN();
  // test_planner();
  // test_door_open_config_trajectory();
 
  return 0;
}