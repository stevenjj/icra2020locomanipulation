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
void Task::setReference(const Eigen::VectorXd & vec_ref){
	std::cout << "Warning! Task " << task_name << " has no setReference(vec_ref)" << " implementation" << std::endl; 
}

void Task::setReference(const Eigen::Quaterniond & quat_ref){
	std::cout << "Warning! Task " << task_name << " has no setReference(quat_ref)" << " implementation" << std::endl;
}

void Task::setReference(const Eigen::VectorXd & vec_ref, const Eigen::Quaterniond & quat_ref){
	std::cout << "Warning! Task " << task_name << " has no setReference(vec_ref, quat_ref)" << " implementation" << std::endl;
}


// Get Task References
void Task::getRef(Eigen::VectorXd & vec_ref_out){
	std::cout << "Warning! Task " << task_name << " has no getRef(vec_ref)" << " implementation" << std::endl;
}

void Task::getRef(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	std::cout << "Warning! Task " << task_name << " has no getRef(vec_ref, quat_ref_out)" << " implementation" << std::endl;
}

void Task::getRef(Eigen::Quaterniond & quat_ref_out){
	std::cout << "Warning! Task " << task_name << " has no getRef(quat_ref_out)" << " implementation" << std::endl;
}



// Computes the error for a given reference
void Task::getError(Eigen::VectorXd & error_out){
	std::cout << "Warning! Task " << task_name << " has no getError(error_out)" << " implementation" << std::endl;
}

// Sets the task error manually
void Task::setError(const Eigen::VectorXd & error){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}