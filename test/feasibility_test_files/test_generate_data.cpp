#include <iostream>
#include <string>
#include <sstream>


#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>

void initialize_config(Eigen::VectorXd & q_init, std::shared_ptr<RobotModel> & robot_model){
  Eigen::VectorXd q_start;
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

void test_generate_and_visualize_N_contact_transition_data(int argc, char ** argv, int N_input, std::string yaml_file, bool visualize, bool generate_positive_data_only){
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
  std::string data_gen_config_filename = std::string(THIS_PACKAGE_PATH) + std::string("data_generation_yaml_configurations/") + yaml_file; 
  feas_data_gen.loadParamFile(data_gen_config_filename);

  // Initialize and start the visualizer
  ros::init(argc, argv, "data_generation");
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(ros_node, robot_model);
  Eigen::VectorXd q_begin = q_ik_start;
  

  // Attempt to generate a contact transition data until success. Visualize each successful result.
  int N_positive_data_to_generate = N_input;

  // whether or not we only want to generate positive examples:
  if (generate_positive_data_only){
	  feas_data_gen.setGenerateOnlyPositiveExamples(true);  	
  }

  bool store_data = true;
  bool visualize_once = true;

  int generated_data_count = 0;
  while(generated_data_count < N_positive_data_to_generate){
    // if we generate a data successfully, increment the counter
    if (feas_data_gen.generateContactTransitionData(store_data)){
      generated_data_count++;
      std::cout << "    Generated positive example # " << generated_data_count << std::endl; 

      if (visualize){
	      std::cout << "  Begin visualizing the trajectory..." << std::endl;
	      // Get q_begin
	      feas_data_gen.ctg->traj_q_config.get_pos(0, q_begin);
	      // Visualize
	      visualizer.visualizeConfigurationTrajectory(q_begin, feas_data_gen.ctg->traj_q_config, visualize_once);
	  }

    }    
  }

}


int main(int argc, char ** argv){
	std::string yaml_filename = "right_hand_left_stance.yaml";
	int N = 1;
    bool visualize = false;   
    bool generate_positive_data_only = false;   

    // for(int i = 0; i < argc; i++){
    // 	std::cout << i << ", " << argv[i] << std::endl;
    // }

	if (argc > 5){
		N = std::stoi(std::string(argv[1]));	
		yaml_filename = std::string(argv[2]);

	    std::stringstream ss_visualize(argv[3]);
	    if (!(ss_visualize >> std::boolalpha >> visualize)){
	    	std::cout << "3rd argument must be either true or false" << std::endl;
	    	return 0;
	    }

		std::stringstream ss_pos_samples_only(argv[4]);
	    if (!(ss_pos_samples_only >> std::boolalpha >> generate_positive_data_only)){
	    	std::cout << "4th argument must be either true or false" << std::endl;
	    	return 0;
	    }

	}else{
		std::cout << "Expected 4 arguments: N(int) yaml_filename(string), visualize(bool) generate_positive_data_only(bool)" << std::endl;
    	std::cout << "Using defaults. Perhaps look at the launch file launch_generate_training_data.launch" << std::endl;
	}

	std::cout << " Generating training data..." << std::endl;
	std::cout << "   N = " << N << " , the number of positive samples to be generated" << std::endl;
	std::cout << "   Loading yaml file: " << yaml_filename << std::endl;
	std::cout << "   visualize =  " << (visualize ? "true" : "false") << std::endl;
	std::cout << "   generate_positive_data_only =  " << (generate_positive_data_only ? "true" : "false") << std::endl;

	test_generate_and_visualize_N_contact_transition_data(argc, argv, N, yaml_filename, visualize, generate_positive_data_only);

}