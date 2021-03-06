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
  // Eigen::VectorXd q_final_door;
  // load_final_robot_door_configuration(q_final_door);

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

  ctg->wpg.setDoubleSupportTime(0.2);
  ctg->wpg.setSettlingPercentage(0.4); // Percentage to settle. Default 0.999
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
  double s_goal = 0.99; //0.20; //0.12;//0.08;
  shared_ptr<Node> starting_vertex (std::make_shared<LMVertex>(s_init, q_start_door));    
  shared_ptr<Node> goal_vertex (std::make_shared<LMVertex>(s_goal));

  lm_planner.setStartNode(starting_vertex);
  lm_planner.setGoalNode(goal_vertex);
  
  bool a_star_success = lm_planner.doAstar();
  if (a_star_success){
    // Construct the path
    // std::cout << "Constructing the dynamic trajectory" << std::endl;
    // std::cout << "Path has size: " << lm_planner.optimal_path.size() << std::endl;
    // if (lm_planner.reconstructConfigurationTrajectoryv2()){
    //   // Visualize the trajectory:
    //   std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
    //   RVizVisualizer visualizer(ros_node, valkyrie_model);  
    // visualizer.visualizeConfigurationTrajectory(q_start_door, lm_planner.path_traj_q_config, lm_planner.s_traj);
    // }else{
    //   std::cout << "non convergence" << std::endl;
    // }

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