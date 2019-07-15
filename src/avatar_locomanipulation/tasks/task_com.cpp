#include <avatar_locomanipulation/tasks/task_com.hpp>

TaskCOM::TaskCOM(std::shared_ptr<RobotModel> & input_model){
	robot_model = input_model;
	task_dim = 3;
	task_name = "CoM Task";
	frame_name = "CoM Frame";
	std::cout << "[Task COM] Constructed" << std::endl;
	cur_pos_.setZero();
	error_ = Eigen::VectorXd::Zero(3);
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


// Set Task References
void TaskCOM::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}

// Get Task References
void TaskCOM::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}

void TaskCOM::computeError(){
	// Get COM
	cur_pos_ = robot_model->x_com;
	// Compute Linear Error
	error_.head(3) = vec_ref_ - cur_pos_;
}

// Computes the error for a given reference
void TaskCOM::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}