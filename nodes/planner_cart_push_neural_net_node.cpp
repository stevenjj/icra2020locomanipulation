#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>
#include "pinocchio/utils/timer.hpp"

// Planner
#include <avatar_locomanipulation/planners/a_star_planner.hpp>
#include <avatar_locomanipulation/planners/locomanipulation_a_star_planner.hpp>


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

using namespace planner;


void load_initial_robot_door_configuration(Eigen::VectorXd & q_out, Eigen::Vector3d & cart_pos_out, Eigen::Quaterniond & cart_ori_out){
  ParamHandler param_handler;
  param_handler.load_yaml_file(THIS_PACKAGE_PATH"stored_configurations/robot_cart_initial_configuration_v1.yaml");  

  // Get the robot configuration vector
  std::vector<double> robot_q;
  param_handler.getVector("robot_starting_configuration", robot_q);
  
  // Get cart location and orientation
  double cart_x, cart_y, cart_z, cart_rx, cart_ry, cart_rz, cart_rw;
  param_handler.getNestedValue({"cart_fixed_frame_position", "x"}, cart_x);
  param_handler.getNestedValue({"cart_fixed_frame_position", "y"}, cart_y);
  param_handler.getNestedValue({"cart_fixed_frame_position", "z"}, cart_z);
  param_handler.getNestedValue({"cart_fixed_frame_orientation", "x"}, cart_rx);
  param_handler.getNestedValue({"cart_fixed_frame_orientation", "y"}, cart_ry);
  param_handler.getNestedValue({"cart_fixed_frame_orientation", "z"}, cart_rz);
  param_handler.getNestedValue({"cart_fixed_frame_orientation", "w"}, cart_rw);

  // Convert to Eigen data types
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_q.size());
  for(int i = 0; i < robot_q.size(); i++){
    q[i] = robot_q[i];
  }
  Eigen::Vector3d cart_position(cart_x, cart_y, cart_z);
  Eigen::Quaterniond cart_orientation(cart_rw, cart_rx, cart_ry, cart_rz);

  // Set outputs
  q_out = q;
  cart_pos_out = cart_position;
  cart_ori_out = cart_orientation;
}


void load_final_robot_object_configuration(Eigen::VectorXd & q_out){
  ParamHandler param_handler;
  param_handler.load_yaml_file(THIS_PACKAGE_PATH"stored_configurations/robot_cart_initial_configuration_v1.yaml");  

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


void test_LM_planner(){
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
  Eigen::Vector3d cart_position;
  Eigen::Quaterniond cart_orientation;
  load_initial_robot_door_configuration(q_start_door, cart_position, cart_orientation);


  // Get the final guessed robot configuration
  // Eigen::VectorXd q_final_door;
  // load_final_robot_object_configuration(q_final_door);

  // Initialize the manipulation function for manipulating the door ----------------------------------
  std::string right_hand_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/cart_right_hand_trajectory.yaml";
  std::string left_hand_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/cart_left_hand_trajectory.yaml";

  // std::shared_ptr<ManipulationFunction> f_s_manipulate_door(new ManipulationFunction(right_hand_yaml_file));
  std::shared_ptr<ManipulationFunction> f_s_manipulate_door(new ManipulationFunction());
  f_s_manipulate_door->setRightWaypointsFromYaml(right_hand_yaml_file);
  f_s_manipulate_door->setLeftWaypointsFromYaml(left_hand_yaml_file);

  // f_s_manipulate_door->setManipulationType(MANIPULATE_TYPE_RIGHT_HAND);
  // f_s_manipulate_door->setManipulationType(MANIPULATE_TYPE_LEFT_HAND);
  f_s_manipulate_door->setManipulationType(MANIPULATE_TYPE_BOTH_HANDS);


  // Set cart location as waypoints are with respect to the cart
  cart_orientation.normalize();
  f_s_manipulate_door->setWorldTransform(cart_position, cart_orientation);

  // Initialize Config Trajectory Generator
  int N_resolution = 60; //single step = 30. two steps = 60// 30* number of footsteps
  std::shared_ptr<ConfigTrajectoryGenerator> ctg(new ConfigTrajectoryGenerator(valkyrie_model, N_resolution));
  valkyrie_model->updateFullKinematics(q_start_door);

  // Initialize Trajectory Generation Module
  ctg->setUseRightHand(true);
  ctg->setUseLeftHand(true);
  ctg->setUseTorsoJointPosition(true);
  ctg->setUseArmLowerPriorityTask(true);
  // ctg->setUseTorsoJointPosition(false);
  ctg->reinitializeTaskStack();
  // timer
  //PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);

  ctg->wpg.setDoubleSupportTime(0.2);
  ctg->wpg.setSettlingPercentage(0.4); // Percentage to settle. Default 0.999
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


  // Set discretization of delta_s
  lm_planner.delta_s_max = 0.04;
  lm_planner.w_distance = 2000;
  lm_planner.generateDiscretization();


  double s_init = 0.0;
  double s_goal = 0.4; //0.20; //0.12;//0.08;
  shared_ptr<Node> starting_vertex (std::make_shared<LMVertex>(s_init, q_start_door));    
  shared_ptr<Node> goal_vertex (std::make_shared<LMVertex>(s_goal));

  lm_planner.setStartNode(starting_vertex);
  lm_planner.setGoalNode(goal_vertex);
  
  bool a_star_success = lm_planner.doAstar();
  if (a_star_success){
    // Visualize the trajectory:
    std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
    RVizVisualizer visualizer(ros_node, valkyrie_model);  
    visualizer.visualizeConfigurationTrajectory(q_start_door, lm_planner.path_traj_q_config, lm_planner.s_traj);

  }


}

int main(int argc, char ** argv){   
  ros::init(argc, argv, "test_planner_with_trajectories");

  test_LM_planner();
 
  return 0;
}