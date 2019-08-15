#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>


void initialize_config(Eigen::VectorXd & q_init, std::shared_ptr<RobotModel> & robot_model){
  // X dimensional state vectors
  Eigen::VectorXd q_start;

  // std::cout << "robot_model->getDimQ(): " << robot_model->getDimQ() << std::endl;
  // Set origin at 0.0
  q_start = Eigen::VectorXd::Zero(robot_model->getDimQ());

  double theta = 0.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q
  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[robot_model->getJointIndex("leftHipPitch")] = -0.3;
  q_start[robot_model->getJointIndex("rightHipPitch")] = -0.3;
  q_start[robot_model->getJointIndex("leftKneePitch")] = 0.6;
  q_start[robot_model->getJointIndex("rightKneePitch")] = 0.6;
  q_start[robot_model->getJointIndex("leftAnklePitch")] = -0.3;
  q_start[robot_model->getJointIndex("rightAnklePitch")] = 0.0;//-0.3;

  q_start[robot_model->getJointIndex("rightShoulderPitch")] = 0.2;
  q_start[robot_model->getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[robot_model->getJointIndex("rightElbowPitch")] = 1.0 ; //0.4;
  q_start[robot_model->getJointIndex("rightForearmYaw")] = 1.5;

  q_start[robot_model->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[robot_model->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[robot_model->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[robot_model->getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}

void test_initial_configuration_data_generation(int argc, char ** argv){
  std::cout << "[Testing FeasibilityDataGenerator]" << std::endl;

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));
  Eigen::VectorXd q_ik_start;
  initialize_config(q_ik_start, robot_model);

  // Initialize feasibility data generator
  FeasibilityDataGenerator feas_data_gen;

  // Set the robot model
  feas_data_gen.setRobotModel(robot_model);
  // Set the initial IK configuration
  feas_data_gen.setStartingIKConfig(q_ik_start);

  // Set the seed
  unsigned int seed_number = 1;
  feas_data_gen.initializeSeed(seed_number);

  // Try finding initial configurations
  int N_tries = 100;
  bool found_start_config = false;

  int N_success = 0;
  std::vector<Eigen::VectorXd> q_successes;

  for(int i = 0; i < N_tries; i++){
    // set random starting configuration
    found_start_config = feas_data_gen.randomizeStartingConfiguration();   
    std::cout << "Found starting config? " << (found_start_config ? "True" : "False") << std::endl;
    if (found_start_config){
      std::cout << "randomizing foot landing configuration:" << std::endl;
      feas_data_gen.randomizeFootLandingConfiguration();

      std::cout << "q_start = " << feas_data_gen.q_start.transpose() << std::endl;
      // Store successful configurations
      q_successes.push_back(feas_data_gen.q_start);
      N_success++;
    }
  }

  // Set the "trajectory" to visualize
  TrajEuclidean   traj_q_config;
  double dt_visualize = 0.5;
  traj_q_config.set_dim_N_dt(robot_model->getDimQ(), N_success, dt_visualize);
  for (int i = 0; i < N_success; i++){
    traj_q_config.set_pos(i, q_successes[i]);
  }


}


void test_generate_contact_transition_data(int argc, char ** argv){
  std::cout << "[Testing FeasibilityDataGenerator]" << std::endl;

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));
  Eigen::VectorXd q_ik_start;
  initialize_config(q_ik_start, robot_model);

  // Initialize feasibility data generator
  FeasibilityDataGenerator feas_data_gen;

  // Set the robot model
  feas_data_gen.setRobotModel(robot_model);
  // Set the initial IK configuration
  feas_data_gen.setStartingIKConfig(q_ik_start);

  // set the data data_gen_config_filename configuration file path
  std::string data_gen_config_filename = THIS_PACKAGE_PATH"data_generation_yaml_configurations/right_hand_left_stance.yaml";
  // std::string data_gen_config_filename = THIS_PACKAGE_PATH"data_generation_yaml_configurations/both_hands_left_stance.yaml";
  feas_data_gen.loadParamFile(data_gen_config_filename);

  // Attempt to generate a contact transition data until success.
  bool store_data = true;
  while(feas_data_gen.generateContactTransitionData(store_data) != true){
    continue;
  }


  // Visualize the successful trajectory
  // Get the starting configuration of the trajectory
  std::cout << "Getting the starting position" << std::endl;
  Eigen::VectorXd q_begin = q_ik_start;
  feas_data_gen.ctg->traj_q_config.get_pos(0, q_begin);

  // Initialize the ros node
  ros::init(argc, argv, "test_feasibility_data_generation");
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  // Initialize and start the visualizer
  RVizVisualizer visualizer(ros_node, robot_model);

  std::cout << "Begin visualizing the trajectory..." << std::endl;
  std::cout << "size of traj_q_config = " << feas_data_gen.ctg->traj_q_config.get_trajectory_length() << std::endl;  
  visualizer.visualizeConfigurationTrajectory(q_begin, feas_data_gen.ctg->traj_q_config);
}

void test_generate_N_contact_transition_data(int argc, char ** argv){
  std::cout << "[Testing FeasibilityDataGenerator]" << std::endl;

  // Initialize the robot model and the initial joint configuration
  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));
  Eigen::VectorXd q_ik_start;
  initialize_config(q_ik_start, robot_model);

  // Initialize feasibility data generator
  FeasibilityDataGenerator feas_data_gen;
  // Set the robot model
  feas_data_gen.setRobotModel(robot_model);
  // Set the initial IK configuration
  feas_data_gen.setStartingIKConfig(q_ik_start);

  // set the data data_gen_config_filename configuration file path
  std::string data_gen_config_filename = THIS_PACKAGE_PATH"data_generation_yaml_configurations/right_hand_left_stance.yaml";
  feas_data_gen.loadParamFile(data_gen_config_filename);

  // Attempt to generate a contact transition data until success.
  int N_positive_data_to_generate = 2;
  bool store_data = true;
  feas_data_gen.generateNDataTransitions(N_positive_data_to_generate, store_data);

}


void test_generate_and_visualize_N_contact_transition_data(int argc, char ** argv){
  std::cout << "[Testing FeasibilityDataGenerator]" << std::endl;

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));
  Eigen::VectorXd q_ik_start;
  initialize_config(q_ik_start, robot_model);

  // Initialize feasibility data generator
  FeasibilityDataGenerator feas_data_gen;

  // Set the robot model
  feas_data_gen.setRobotModel(robot_model);
  // Set the initial IK configuration
  feas_data_gen.setStartingIKConfig(q_ik_start);

  // set the data data_gen_config_filename configuration file path
  std::string data_gen_config_filename = THIS_PACKAGE_PATH"data_generation_yaml_configurations/right_hand_left_stance.yaml";
  // std::string data_gen_config_filename = THIS_PACKAGE_PATH"data_generation_yaml_configurations/both_hands_left_stance.yaml";
  feas_data_gen.loadParamFile(data_gen_config_filename);


  // Initialize and start the visualizer
  ros::init(argc, argv, "test_feasibility_data_generation");
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(ros_node, robot_model);
  Eigen::VectorXd q_begin = q_ik_start;


  // Attempt to generate a contact transition data until success. Visualize each successful result.
  int N_positive_data_to_generate = 2;
  bool store_data = true;
  bool visualize_once = true;

  int generated_data_count = 0;
  while(generated_data_count < N_positive_data_to_generate){
    // if we generate a data successfully, increment the counter
    if (feas_data_gen.generateContactTransitionData(store_data)){
      generated_data_count++;
      std::cout << "    Generated positive example # " << generated_data_count << std::endl; 

      std::cout << "  Begin visualizing the trajectory..." << std::endl;
      // Get q_begin
      feas_data_gen.ctg->traj_q_config.get_pos(0, q_begin);
      // Visualize
      visualizer.visualizeConfigurationTrajectory(q_begin, feas_data_gen.ctg->traj_q_config, visualize_once);

    }    
  }

}

int main(int argc, char ** argv){
  // test_initial_configuration_data_generation(argc, argv);
  // test_generate_contact_transition_data(argc, argv);
  // test_generate_N_contact_transition_data(argc, argv);
  test_generate_and_visualize_N_contact_transition_data(argc, argv);
  return 0;
}