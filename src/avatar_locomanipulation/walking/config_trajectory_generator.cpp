#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>

// Constructor
ConfigTrajectoryGenerator::ConfigTrajectoryGenerator(){	
}

ConfigTrajectoryGenerator::ConfigTrajectoryGenerator(std::shared_ptr<RobotModel> & robot_model_in){
	this->setRobotModel(robot_model_in);
	initializeDiscretization(100);
}


// Destructor
ConfigTrajectoryGenerator::~ConfigTrajectoryGenerator(){
	std::cout << "[ConfigTrajectoryGenerator] Destroyed" << std::endl;
}


void ConfigTrajectoryGenerator::setRobotModel(std::shared_ptr<RobotModel> & robot_model_in){
	robot_model = robot_model_in;
	ik_module.setRobotModel(robot_model_in);
}

void ConfigTrajectoryGenerator::initializeDiscretization(const int & N_size_in){
	N_size = N_size_in;

	// Set a dummy dt. 
	double dt_dummy = 1e-3;
	// Initialize rajectory sizes 
	traj_q_config.set_dim_N_dt(robot_model->getDimQ(), N_size, dt_dummy);
	traj_SE3_left_hand.set_N_dt(N_size_in, dt_dummy);
	traj_SE3_right_hand.set_N_dt(N_size_in, dt_dummy);
	wpg.initialize_trajectory_discretization(N_size);

}


int ConfigTrajectoryGenerator::getDiscretizationSize(){
	return N_size;
}

void ConfigTrajectoryGenerator::computeInitialConfigForFlatGround(const Eigen::VectorXd & q_guess, Eigen::VectorXd & q_out){	
}






// construct_trajectories(const std::vector<Footstep> & input_footstep_list, 
//                                                      const Footstep & initial_left_footstance,
//                                                      const Footstep & initial_right_footstance, 
//                                                      const Eigen::Vector3d & initial_com,
//                                                      const Eigen::Quaterniond initial_pelvis_ori)