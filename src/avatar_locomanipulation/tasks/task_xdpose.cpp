#include <avatar_locomanipulation/tasks/task_xdpose.hpp>

TaskXDPose::TaskXDPose(std::shared_ptr<RobotModel> & input_model, 
					   std::vector<int> input_task_dimensions,
					   const std::string & input_frame_name): Task6DPose(input_model, input_frame_name){
	if (task_dimensions.size() > 6){
		std::cout << "[TaskXDPose] Error. Task Dimensions must be at most 6." << std::endl;
		throw;
	}

	task_dimensions = input_task_dimensions;
	task_dim = task_dimensions.size();

	error_tmp = Eigen::VectorXd::Zero(6);
	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	// We will only compute Jacobians and errors for the task dimension specified
	error_ = Eigen::VectorXd::Zero(task_dim);
	J_out = Eigen::MatrixXd::Zero(task_dim, robot_model->getDimQdot());
	Jdot_out = Eigen::MatrixXd::Zero(task_dim, robot_model->getDimQdot());

	// std::cout << "[TaskXDPose] for frame " << frame_name << " Constructed" << std::endl;
}

TaskXDPose::~TaskXDPose(){
	// std::cout << "[TaskXDPose] for frame " << frame_name << " Destroyed" << std::endl;
}

void TaskXDPose::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	for(int i = 0 ; i < task_dimensions.size(); i++){
		J_out.row(i) = J_tmp.row(task_dimensions[i]); 
	}

	J_task = J_out;
}
void TaskXDPose::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	for(int i = 0 ; i < task_dimensions.size(); i++){
		Jdot_out.row(i) = Jdot_tmp.row(task_dimensions[i]); 
	}
	Jdot_task = Jdot_out;

}

void TaskXDPose::computeError(){
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);
	// Compute Linear Error
	error_tmp.head(3) = kp_task_gain_*(vec_ref_ - cur_pos_);
	// Compute Quaternion Error
	math_utils::compute_quat_error(quat_ref_, quat_current_, quat_error_);
	error_tmp.tail(3) = kp_task_gain_*(quat_error_);	

	// Output the error
	for(int i = 0 ; i < task_dimensions.size(); i++){
		error_[i] = error_tmp[task_dimensions[i]];
	}

}
