#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>
// Standard
#include <iostream>
#include <math.h>
#include <cassert>


void initialize_config(Eigen::VectorXd & q_init, std::shared_ptr<RobotModel> & valkyrie){
  // X dimensional state vectors
  Eigen::VectorXd q_start;

  std::cout << "valkyrie->getDimQ(): " << valkyrie->getDimQ() << std::endl;
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

void test_config_trajectory_generator(){
  std::cout << "[Running Config Trajectory Generator Test]" << std::endl;

  std::cout << "[ConfigTrajectoryGenerator] Constructed" << std::endl;
  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  int N_resolution = 150;
  ConfigTrajectoryGenerator ctg(valkyrie_model, N_resolution);

  Eigen::VectorXd q_start, q_end;
  initialize_config(q_start, valkyrie_model);

  // Test initial configuration computation for flat ground
  ctg.computeInitialConfigForFlatGround(q_start, q_end);


  // Visualize the robot
  std::shared_ptr<ros::NodeHandle> node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(node, valkyrie_model);  
  // visualizer.visualizeConfiguration(q_start, q_end);

  // Update Initial
  q_start = q_end;
  valkyrie_model->updateFullKinematics(q_start);

  // Create footsteps in place
  Footstep footstep_1; footstep_1.setLeftSide();
  Footstep footstep_2; footstep_2.setRightSide();
  Footstep footstep_3; footstep_3.setLeftSide();
  Footstep footstep_4; footstep_4.setRightSide();
  valkyrie_model->getFrameWorldPose("leftCOP_Frame", footstep_1.position, footstep_1.orientation);  
  valkyrie_model->getFrameWorldPose("rightCOP_Frame", footstep_2.position, footstep_2.orientation);  

  footstep_3 = footstep_1;
  footstep_4 = footstep_2;

  // Walk forward 4 steps
  // footstep_1.position[0] += 0.15;
  // footstep_2.position[0] += 0.25;

  // footstep_3.position[0] += 0.35;
  // footstep_4.position[0] += 0.35;


  // footstep_1.position[0] += 0.15;
  // footstep_2.position[0] += 0.30;

  // footstep_3.position[0] += 0.45;
  // footstep_4.position[0] += 0.45;

  footstep_1.position[0] += 0.20;
  footstep_2.position[0] += 0.40;

  footstep_3.position[0] += 0.60;
  footstep_4.position[0] += 0.60;


  // Create Footstep list
  std::vector<Footstep> input_footstep_list = {footstep_1, footstep_2, footstep_3, footstep_4};

  // Solve for configurations
  ctg.computeConfigurationTrajectory(q_start, input_footstep_list);

  // Visualize Trajectory
  visualizer.visualizeConfigurationTrajectory(q_start, ctg.traj_q_config);

}

int main(int argc, char ** argv){   
  ros::init(argc, argv, "test_config_trajectory_generator");
  test_config_trajectory_generator();


  return 0;
}