#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>

// Constructor
ConfigTrajectoryGenerator::ConfigTrajectoryGenerator(){	
}

ConfigTrajectoryGenerator::ConfigTrajectoryGenerator(std::shared_ptr<RobotModel> & robot_model_in){
	this->setRobotModel(robot_model_in);
	initializeDiscretization(100);
	initializeTasks();
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


void ConfigTrajectoryGenerator::initializeTasks(){
	pelvis_ori_task = std::shared_ptr<Task>(new Task3DOrientation(robot_model, "pelvis"));
	com_task = std::shared_ptr<Task>(new TaskCOM(robot_model));

	rfoot_task = std::shared_ptr<Task>(new Task6DPose(robot_model, "rightCOP_Frame"));
	lfoot_task = std::shared_ptr<Task>(new Task6DPose(robot_model, "leftCOP_Frame"));

	rhand_task = std::shared_ptr<Task>(new Task6DPose(robot_model, "rightPalm"));
	lhand_task = std::shared_ptr<Task>(new Task6DPose(robot_model, "leftPalm"));

    std::vector<std::string> torso_joint_names = {"torsoYaw", "torsoPitch", "torsoRoll"};
    std::vector<std::string> neck_joint_names = {"lowerNeckPitch", "neckYaw", "upperNeckPitch"};
    std::vector<std::string> left_arm_joint_names = {"leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch"};
    std::vector<std::string> right_arm_joint_names = {"rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch"};

    torso_posture_task = std::shared_ptr<Task>(new TaskJointConfig(robot_model, torso_joint_names));
    neck_posture_task = std::shared_ptr<Task>(new TaskJointConfig(robot_model, neck_joint_names));
    rarm_posture_task = std::shared_ptr<Task>(new TaskJointConfig(robot_model, left_arm_joint_names));
    larm_posture_task = std::shared_ptr<Task>(new TaskJointConfig(robot_model, right_arm_joint_names));
}


void ConfigTrajectoryGenerator::createTaskStack(){
	std::vector< std::shared_ptr<Task> > vec_task_stack = {pelvis_ori_task, com_task, lfoot_task, rfoot_task, neck_posture_task};

	// Check whether to use a 	
	if (use_right_hand){
		vec_task_stack.push_back(rhand_task);
	}else{
		vec_task_stack.push_back(rarm_posture_task);
	}

	if (use_left_hand){
		vec_task_stack.push_back(lhand_task);
	}else{
		vec_task_stack.push_back(larm_posture_task);
	}

	if (use_torso_joint_position){
		vec_task_stack.push_back(torso_posture_task);	
	}


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