#include <avatar_locomanipulation/tasks/task_main.hpp>

TaskMain::TaskMain(){}
TaskMain::TaskMain(std::shared_ptr<ValkyrieModel> & input_model){
	robot_model = input_model;
	std::cout << "[Task Main] Constructed" << std::endl;
}

TaskMain::~TaskMain(){
	std::cout << "[Task Main] Destroyed" << std::endl;
}

void TaskMain::getTaskJacobian(Eigen::MatrixXd & J_task){
	
}
void TaskMain::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	
}
