#include <avatar_locomanipulation/tasks/task_6dpose_no_rxry.hpp>

Task6DPoseNoRXRY::Task6DPoseNoRXRY(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name){
	robot_model = input_model;
	task_dim = 6;
	task_name = input_frame_name;
	frame_name = input_frame_name;

	error_ = Eigen::VectorXd::Zero(task_dim);
	vec_ref_ = Eigen::VectorXd::Zero(3);
	quat_ref_.setIdentity();

	// std::cout << "[Task 6D Pose] for frame " << frame_name << " Constructed" << std::endl;
}

Task6DPoseNoRXRY::~Task6DPoseNoRXRY(){
	// std::cout << "[Task 6D Pose] for frame " << frame_name << " Destroyed" << std::endl;
}

void Task6DPoseNoRXRY::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_task);
	// Remove rx, ry contributions of the floating base
	// J_task.col(2) = Eigen::VectorXd::Zero(6) ;
	J_task.col(3) = Eigen::VectorXd::Zero(6) ;
	J_task.col(4) = Eigen::VectorXd::Zero(6) ;

}

void Task6DPoseNoRXRY::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_task);
	// Remove rx, ry contributions of the floating base
	// Jdot_task.col(2) = Eigen::VectorXd::Zero(6);
	Jdot_task.col(3) = Eigen::VectorXd::Zero(6);
	Jdot_task.col(4) = Eigen::VectorXd::Zero(6);
}

// Set Task References
void Task6DPoseNoRXRY::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}

void Task6DPoseNoRXRY::setReference(const Eigen::Quaterniond & quat_ref_in){
	quat_ref_ = quat_ref_in;
}


void Task6DPoseNoRXRY::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	vec_ref_ = vec_ref_in;
	quat_ref_ = quat_ref_in;
}


// Get Task References
void Task6DPoseNoRXRY::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}

void Task6DPoseNoRXRY::getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	vec_ref_out = vec_ref_;
	quat_ref_out = quat_ref_;
}

void Task6DPoseNoRXRY::getReference(Eigen::Quaterniond & quat_ref_out){
	quat_ref_out = quat_ref_;
}

void Task6DPoseNoRXRY::computeError(){
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);
	// Compute Linear Error
	error_.head(3) = kp_task_gain_*(vec_ref_ - cur_pos_);
	// Compute Quaternion Error
	math_utils::compute_quat_error(quat_ref_, quat_current_, quat_error_);
	error_.tail(3) = kp_task_gain_*(quat_error_);	
}

// Computes the error for a given reference
void Task6DPoseNoRXRY::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

// Sets the task error manually
void Task6DPoseNoRXRY::setError(const Eigen::VectorXd & error_in){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}