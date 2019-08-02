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

  // Get the Initial Robot and Door Configuration
  Eigen::VectorXd q_start_door;
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

  std::cout << "Converged?" << (convergence ? "True" : "False") << std::endl; 

  std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;
  // Visualize Trajectory
  visualizer.visualizeConfigurationTrajectory(q_start_door, ctg.traj_q_config);

}


void test_LM_planner(){
  // Initialize the robot ---------------------------------------------------------------------------------------
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));
  // Get the Initial Robot and Door Configuration
  Eigen::VectorXd q_start_door;
  Eigen::Vector3d hinge_position;
  Eigen::Quaterniond hinge_orientation;
  load_robot_door_configuration(q_start_door, hinge_position, hinge_orientation);

  // Initialize the manipulation function for manipulating the door ----------------------------------
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_trajectory.yaml";
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
  ctg->setUseTorsoJointPosition(false);
  ctg->reinitializeTaskStack();
  // timer
  //PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);

  // Have fast double support times
  ctg->wpg.setDoubleSupportTime(0.2);
  // Set Manipulation Only time to 3 seconds
  ctg->setManipulationOnlyTime(3.0);
  // Set Verbosity
  ctg->setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2);

  // End of initialization ---------------------------------------------------------------------------------------

  LocomanipulationPlanner lm_planner;
  // Initialize
  lm_planner.initializeLocomanipulationVariables(valkyrie_model, f_s_manipulate_door, ctg);

  shared_ptr<Node> starting_vertex (std::make_shared<LMVertex>());    
  shared_ptr<Node> goal_vertex (std::make_shared<LMVertex>());

  lm_planner.setStartNode(starting_vertex);
  lm_planner.setGoalNode(goal_vertex);
  lm_planner.doAstar();
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
  test_LM_planner();

  // test_planner();
  // ros::init(argc, argv, "test_planner_with_trajectories");
  // test_door_open_config_trajectory();
 
  return 0;
}