#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>

FeasibilityDataGenerator::FeasibilityDataGenerator(){
	initialize_modules();
}

FeasibilityDataGenerator::~FeasibilityDataGenerator(){
}

void FeasibilityDataGenerator::initialize_modules(){
	ik_start_config_module.reset(new IKModule());
	ctg.reset(new ConfigTrajectoryGenerator());
}

void FeasibilityDataGenerator::setRobotModel(std::shared_ptr<RobotModel> & robot_model_in){
	robot_model = robot_model_in;
	ik_start_config_module->setRobotModel(robot_model_in);
	ctg->setRobotModel(robot_model_in);
	ctg->commonInitialization();
}
