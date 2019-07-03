#include <avatar_locomanipulation/tasks/task_com.hpp>

TaskCOM::TaskCOM(std::shared_ptr<ValkyrieModel> & input_model){
	robot_model = input_model;
	task_dim = 3;
	task_name = "CoM Task";
	frame_name = "CoM Frame";
	std::cout << "[Task COM] Constructed" << std::endl;
}

TaskCOM::~TaskCOM(){
	std::cout << "[Task COM] Destroyed" << std::endl;
}

void TaskCOM::getTaskJacobian(Eigen::MatrixXd & J_task){
	J_task = robot_model->J_com;
}
void TaskCOM::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	Jdot_task = robot_model->Jdot_com;	
}
