#include <avatar_locomanipulation/tasks/task_stack.hpp>

TaskStack::TaskStack(std::shared_ptr<ValkyrieModel> & input_model, const std::vector< std::shared_ptr<Task> > & task_list_input){
	robot_model = input_model;
	task_list = task_list_input;

	task_dim = 0;
	task_name = "Task Stack: ";
	for(int i = 0; i < task_list.size(); i++){
		task_dim += task_list[i]->task_dim;
		task_name = task_name + task_list[i]->task_name + ", ";
		vec_Jtmp.push_back( Eigen::MatrixXd::Zero(task_list[i]->task_dim, robot_model->getDimQdot()) );
		vec_Jdottmp.push_back( Eigen::MatrixXd::Zero(task_list[i]->task_dim, robot_model->getDimQdot()) );
	}
	
	J_stack = Eigen::MatrixXd::Zero(task_dim, robot_model->getDimQdot());
	Jdot_stack = Eigen::MatrixXd::Zero(task_dim, robot_model->getDimQdot());

	// Set size of error
	error_ = Eigen::VectorXd::Zero(task_dim);

	std::cout << "[Task Stack] for tasks " << task_name << " Constructed" << std::endl;
}

TaskStack::~TaskStack(){
	// std::cout << "[Task Stack] for tasks " << task_name << " Destroyed" << std::endl;
}

void TaskStack::getTaskJacobian(Eigen::MatrixXd & J_task){
	int index = 0;
	for(int i = 0; i < task_list.size(); i++){
		task_list[i]->getTaskJacobian(vec_Jtmp[i]);
		J_stack.block(index, 0, task_list[i]->task_dim, robot_model->getDimQdot()) = vec_Jtmp[i];		
		index += task_list[i]->task_dim;		
	}
	J_task = J_stack;
}
void TaskStack::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	int index = 0;
	for(int i = 0; i < task_list.size(); i++){
		task_list[i]->getTaskJacobianDot(vec_Jdottmp[i]);
		Jdot_stack.block(index, 0, task_list[i]->task_dim, robot_model->getDimQdot()) = vec_Jdottmp[i];		
		index += task_list[i]->task_dim;		
	}
	Jdot_task = Jdot_stack;
}


// Computes the error for a given reference
void TaskStack::computeError(){
	int index = 0;
	Eigen::VectorXd sub_task_error;
	for(int i = 0; i < task_list.size(); i++){
		sub_task_error = Eigen::VectorXd::Zero(task_list[i]->task_dim);
		task_list[i]->getError(sub_task_error);
		error_.segment(index, task_list[i]->task_dim) = sub_task_error;
		index += task_list[i]->task_dim;
	}	
}

// Gets the current error
void TaskStack::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

