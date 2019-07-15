#include <avatar_locomanipulation/tasks/task_6dpose.hpp>

Task6DPose::Task6DPose(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name){
	robot_model = input_model;
	task_dim = 6;
	task_name = input_frame_name;
	frame_name = input_frame_name;

	error_ = Eigen::VectorXd::Zero(task_dim);
	vec_ref_ = Eigen::VectorXd::Zero(3);
	quat_ref_.setIdentity();

	std::cout << "[Task 6D Pose] for frame " << frame_name << " Constructed" << std::endl;
}

Task6DPose::~Task6DPose(){
	// std::cout << "[Task 6D Pose] for frame " << frame_name << " Destroyed" << std::endl;
}

void Task6DPose::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_task);
}
void Task6DPose::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_task);
}



// Set Task References
void Task6DPose::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}

void Task6DPose::setReference(const Eigen::Quaterniond & quat_ref_in){
	quat_ref_ = quat_ref_in;
}


void Task6DPose::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	vec_ref_ = vec_ref_in;
	quat_ref_ = quat_ref_in;
}


// Get Task References
void Task6DPose::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}

void Task6DPose::getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	vec_ref_out = vec_ref_;
	quat_ref_out = quat_ref_;
}

void Task6DPose::getReference(Eigen::Quaterniond & quat_ref_out){
	quat_ref_out = quat_ref_;
}

void Task6DPose::computeError(){
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);
	// Compute Linear Error
	error_.head(3) = kp_task_gain_*(vec_ref_ - cur_pos_);
	// Compute Quaternion Error
	math_utils::compute_quat_error(quat_ref_, quat_current_, quat_error_);
	error_.tail(3) = kp_task_gain_*(quat_error_);	
}

// Computes the error for a given reference
void Task6DPose::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

// Sets the task error manually
void Task6DPose::setError(const Eigen::VectorXd & error_in){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}