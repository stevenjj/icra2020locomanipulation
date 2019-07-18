#include <avatar_locomanipulation/tasks/task_6dcontact_normal.hpp>

Task6DContactNormalTask::Task6DContactNormalTask(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name,
												 const Eigen::Vector3d & normal_vec, const Eigen::Vector3d normal_vec_tail): 
	Task6DPose(input_model, input_frame_name){
	normal_vec_hat_ = normal_vec.normalized();
	normal_vec_tail_ = normal_vec_tail;

	std::cout << "normal_vec_hat_ = " << normal_vec_hat_.transpose() << std::endl;
	std::cout << "normal_vec_tail_ = " << normal_vec_tail_.transpose() << std::endl;

	R_frame_ori_.setIdentity();
	z_hat_.setZero();
	omega_hat_.setZero();
	theta_angle_ = 0.0;
	dist_to_plane = 0.0;
	omega_ = Eigen::AngleAxisd(theta_angle_, Eigen::Vector3d(0,0,1));
	quat_omega_.setIdentity();

	des_pos.setZero();
	des_quat.setIdentity();
	std::cout << "[Task6DContactNormalTask] for frame " << frame_name << " Constructed" << std::endl;
}

Task6DContactNormalTask::~Task6DContactNormalTask(){
	// std::cout << "[Task6DContactNormalTask] for frame " << frame_name << " Destroyed" << std::endl;
}

void Task6DContactNormalTask::computeError(){
	// Get current position of the end-effector
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);
	
	// Compute signed perpendicular distance to the plane.
	// v = end_effector - plane_center
	// distance = v \cdot normal_vec_hat
	dist_to_plane = (cur_pos_ - normal_vec_tail_).dot(normal_vec_hat_);

	// Compute the desired position, which is the point on the plane closest to the end effector link
	des_pos = cur_pos_ - normal_vec_hat_*dist_to_plane;

	// Get z hat vector of the end effector frame
	R_frame_ori_ = quat_current_.toRotationMatrix();
	z_hat_ = R_frame_ori_.col(2); // Third column of the rotation matrix

	// Compute orientation error (zhat) to normal vector zhat.
	theta_angle_ = acos(z_hat_.dot(normal_vec_hat_));
	omega_hat_ =  z_hat_.cross(normal_vec_hat_);
	omega_hat_.normalize();
	omega_.angle() = theta_angle_;
	omega_.axis() = omega_hat_;
	// std::cout << "Axis " << omega_.axis().transpose() << ", Angle  " << omega_.angle() << std::endl;  

	// Find desired quaternion
	quat_omega_ = omega_;
	math_utils::printQuat(quat_omega_);
	des_quat = quat_omega_*quat_current_;

	// Compute Errors
	// Compute Linear Error
	error_.head(3) = kp_task_gain_*(des_pos - cur_pos_);
	// Compute Quaternion Error
	error_.tail(3) = kp_task_gain_*(omega_.angle()*omega_.axis());	
}

void Task6DContactNormalTask::setReference(const Eigen::VectorXd & vec_ref_in){
	std::cout << "[Task6DContactNormalTask] Warning. setReference is not implemented for this task " << std::endl;	
}
void Task6DContactNormalTask::setReference(const Eigen::Quaterniond & quat_ref_in){
	std::cout << "[Task6DContactNormalTask] Warning. setReference is not implemented for this task " << std::endl;	
}
void Task6DContactNormalTask::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	std::cout << "[Task6DContactNormalTask] Warning. setReference is not implemented for this task " << std::endl;	
}
