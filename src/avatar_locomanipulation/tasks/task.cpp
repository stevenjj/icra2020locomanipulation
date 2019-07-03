#include <avatar_locomanipulation/tasks/task.hpp>

Task::Task(){}
Task::Task(std::shared_ptr<ValkyrieModel> & input_model){
	robot_model = input_model;
	std::cout << "[Task] Constructed" << std::endl;
}

Task::~Task(){
	std::cout << "[Task] " << task_name << " Destroyed" << std::endl;
}

void Task::getTaskJacobian(Eigen::MatrixXd & J_task){
	
}
void Task::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	
}
