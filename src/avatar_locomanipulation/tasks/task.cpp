#include <avatar_locomanipulation/tasks/task.hpp>

Task::Task(){}
Task::Task(std::shared_ptr<ValkyrieModel> & input_model){
	robot_model = input_model;
	std::cout << "[Task] Constructed" << std::endl;
}

Task::~Task(){
	std::cout << "[Task] " << task_name << " Destroyed" << std::endl;
}

// Get Task Jacobians
void Task::getTaskJacobian(Eigen::MatrixXd & J_task){
	std::cout << "Warning! Task " << task_name << " has no getTaskJacobian(J_task)" << " implementation" << std::endl;	
}
void Task::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	std::cout << "Warning! Task " << task_name << " has no getTaskJacobianDot(Jdot_task)" << " implementation" << std::endl;		
}

// Set Task References
void Task::setReference(const Eigen::VectorXd & vec_ref_in){
	std::cout << "Warning! Task " << task_name << " has no setReference(vec_ref_in)" << " implementation" << std::endl; 
}

void Task::setReference(const Eigen::Quaterniond & quat_ref_in){
	std::cout << "Warning! Task " << task_name << " has no setReference(quat_ref_in)" << " implementation" << std::endl;
}

void Task::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	std::cout << "Warning! Task " << task_name << " has no setReference(vec_ref_in, quat_ref_in)" << " implementation" << std::endl;
}


// Get Task References
void Task::getReference(Eigen::VectorXd & vec_ref_out){
	std::cout << "Warning! Task " << task_name << " has no getReference(vec_ref_out)" << " implementation" << std::endl;
}

void Task::getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	std::cout << "Warning! Task " << task_name << " has no getReference(vec_ref_out, quat_ref_out)" << " implementation" << std::endl;
}

void Task::getReference(Eigen::Quaterniond & quat_ref_out){
	std::cout << "Warning! Task " << task_name << " has no getReference(quat_ref_out)" << " implementation" << std::endl;
}


// Computes the error for a given reference
void Task::computeError(){
	std::cout << "Warning! Task " << task_name << " has no computeError()" << " implementation" << std::endl;
}

// gets error
void Task::getError(Eigen::VectorXd & error_out, bool compute){
	std::cout << "Warning! Task " << task_name << " has no getError(error_out,  bool compute)" << " implementation" << std::endl;
}

// Sets the task error manually
void Task::setError(const Eigen::VectorXd & error_in){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}

// Set the Task's kp gain value. Default = 1.0;
void Task::setTaskGain(const double & kp_task_gain_in){
	kp_task_gain_ = kp_task_gain_in;
}