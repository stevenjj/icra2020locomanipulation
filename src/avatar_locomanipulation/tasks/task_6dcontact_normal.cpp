#include <avatar_locomanipulation/tasks/task_6dcontact_normal.hpp>

Task6DContactNormalTask::Task6DContactNormalTask(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name,
												 const Eigen::Vector3d & normal_vec, const Eigen::Vector3d normal_vec_tail): 
	Task6DPose(input_model, input_frame_name){
	normal_vec_hat_ = normal_vec.normalized();
	normal_vec_tail_ = normal_vec_tail;

	std::cout << "normal_vec_hat_ = " << normal_vec_hat_.transpose() << std::endl;
	std::cout << "normal_vec_tail_ = " << normal_vec_tail_.transpose() << std::endl;

	dist_to_plane = 0.0;
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
	
	std::cout << " task 6d contact normal" << std::endl;
	std::cout << "cur_pos = " << cur_pos_.transpose() << std::endl;
	// math_utils::printQuat(quat_current_);

	// Compute perpendicular distance to the plane.
	// v = end_effector - plane_center
	// distance = v \cdot normal_vec_hat
	dist_to_plane = (cur_pos_ - normal_vec_tail_).dot(normal_vec_hat_);

	std::cout << "dist_to_plane " << dist_to_plane << std::endl;

	// Compute the desired position, which is to move the end effector towards the plane
	des_pos = cur_pos_ - normal_vec_hat_*dist_to_plane;

	std::cout << "des_pos = " << des_pos.transpose() << std::endl;

	// Compute orientation error (zhat) to normal vector zhat.
	// theta_error = acos(ee_zhat \cdot normal_zhat)
	// omega_hat =  ee_zhat \cross normal_zhat

	des_quat = quat_current_;
	// std::cout << "des_pos = " << des_pos << std::endl;
	// std::cout << "des_quat = ";
	// math_utils::printQuat(des_quat);

	// Compute Errors
	// Compute Linear Error
	error_.head(3) = kp_task_gain_*(des_pos - cur_pos_);

	std::cout << "linear error = " << error_.head(3).transpose() << std::endl;

	// Compute Quaternion Error
	math_utils::compute_quat_error(des_quat, quat_current_, quat_error_);
	error_.tail(3) = kp_task_gain_*(quat_error_);	
}
