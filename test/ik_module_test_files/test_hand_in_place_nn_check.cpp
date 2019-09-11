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

void generate_good_init_config(){
  // Initialize the manipulation function for manipulating the door
  std::string door_yaml_file = THIS_PACKAGE_PATH"hand_trajectory/door_trajectory.yaml";
  ManipulationFunction manipulate_door(door_yaml_file);
  // Set hinge location as waypoints are with respect to the hinge
  // Eigen::Vector3d hinge_position(1.5, -0.5, 1.125);
  // Eigen::Quaterniond hinge_orientation(0.707, 0, 0, 0.707);
  Eigen::Vector3d hinge_position(0.0, 0.0, 0.0);
  Eigen::Quaterniond hinge_orientation(1.0, 0, 0, 0.0);

  hinge_orientation.normalize();
  manipulate_door.setWorldTransform(hinge_position, hinge_orientation);


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

  // get right hand position and orientation.
  Eigen::Vector3d rhand_pos, rhand_door_init_pos;
  Eigen::Quaterniond rhand_ori, rhand_door_init_ori;
  valkyrie_model->getFrameWorldPose("rightPalm", rhand_pos, rhand_ori);  

  double s_init = 0.0;
  manipulate_door.getPose(s_init, rhand_door_init_pos, rhand_door_init_ori);

  // Print 
  std::cout << "rhand_pos = " << rhand_pos.transpose() << std::endl;
  std::cout << "rhand_ori = " << rhand_ori.x() << ", " <<
                                 rhand_ori.y() << ", " << 
                                 rhand_ori.z() << ", " << 
                                 rhand_ori.w() << std::endl;

  std::cout << "rhand_door_init_pos = " << rhand_door_init_pos.transpose() << std::endl;
  std::cout << "rhand_door_init_ori = " << rhand_door_init_ori.x() << ", " <<
                                           rhand_door_init_ori.y() << ", " << 
                                           rhand_door_init_ori.z() << ", " << 
                                           rhand_door_init_ori.w() << std::endl; 

  // Compute error and find new hinge location and orientation
  Eigen::Vector3d dx = rhand_pos - rhand_ori.toRotationMatrix()*(rhand_door_init_ori.toRotationMatrix().transpose()*rhand_door_init_pos);
  Eigen::Quaterniond dquat = rhand_ori*rhand_door_init_ori.inverse();

  std::cout << "dx = " << dx.transpose() << std::endl;
  std::cout << "dquat = " << dquat.x() << ", " <<
                             dquat.y() << ", " << 
                             dquat.z() << ", " << 
                             dquat.w() << std::endl; 

  hinge_position = dx;
  hinge_orientation = dquat;

  manipulate_door.setWorldTransform(hinge_position, hinge_orientation);
  manipulate_door.getPose(s_init, rhand_door_init_pos, rhand_door_init_ori);

  std::cout << "New door handle init location" << std::endl;
  std::cout << "rhand_door_init_pos = " << rhand_door_init_pos.transpose() << std::endl;
  std::cout << "rhand_door_init_ori = " << rhand_door_init_ori.x() << ", " <<
                                           rhand_door_init_ori.y() << ", " << 
                                           rhand_door_init_ori.z() << ", " << 
                                           rhand_door_init_ori.w() << std::endl;

  // Store the data
  YAML::Emitter out;
  out << YAML::BeginMap;
    data_saver::emit_joint_configuration(out, "robot_starting_configuration", q_end);
    data_saver::emit_position(out, "hinge_position", hinge_position);
    data_saver::emit_orientation(out, "hinge_orientation", hinge_orientation);
  out << YAML::EndMap;  
  std::cout << "Here's the output YAML:\n" << out.c_str() << "\n" << std::endl;
  std::ofstream file_output_stream("robot_door_initial_configuration_v2.yaml");
  file_output_stream << out.c_str();

}

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

  footstep_1.position[0] -= 0.4;
  footstep_1.position[1] += 0.1;
  // footstep_2.position[0] -= 0.1;

  footstep_1.setPosOri(footstep_1.position, quat_angleaxis);
  footstep_2.setPosOri(footstep_2.position, quat_angleaxis);

  // std::vector<Footstep> input_footstep_list = {footstep_2, footstep_1};
  std::vector<Footstep> input_footstep_list = {footstep_1};


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

int main(int argc, char ** argv){   
  ros::init(argc, argv, "test_config_trajectory_generator");
  test_hand_in_place_config_trajectory_generator();
  // generate_good_init_config();

  return 0;
}