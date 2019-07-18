#include <avatar_locomanipulation/tasks/task_4dcontact_normal.hpp>

Task4DContactNormalTask::Task4DContactNormalTask(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name, 
					 						     const Eigen::Vector3d & normal_vec, const Eigen::Vector3d normal_vec_tail){
	robot_model = input_model;
	task_dim = 4;
	task_name = input_frame_name;
	frame_name = input_frame_name;

	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	normal_vec_hat_ = normal_vec.normalized();
	normal_vec_tail_ = normal_vec_tail;

	std::cout << "normal_vec_hat_ = " << normal_vec_hat_.transpose() << std::endl;
	std::cout << "normal_vec_tail_ = " << normal_vec_tail_.transpose() << std::endl;

	R_frame_ori_.setIdentity();
	z_hat_.setZero();
	omega_hat_.setZero();
	theta_angle_ = 0.0;
	closest_point_on_plane_.setZero();
	dist_to_plane = 0.0;
	omega_ = Eigen::AngleAxisd(theta_angle_, Eigen::Vector3d(0,0,1));
	quat_omega_.setIdentity();

	des_pos.setZero();
	des_quat.setIdentity();
	std::cout << "[Task4DContactNormalTask] for frame " << frame_name << " Constructed" << std::endl;

	error_ = Eigen::VectorXd::Zero(task_dim);

	std::cout << "[Task4DContactNormalTask] for frame " << frame_name << " Constructed" << std::endl;
}

Task4DContactNormalTask::~Task4DContactNormalTask(){
	// std::cout << "[Task4DContactNormalTask] for frame " << frame_name << " Destroyed" << std::endl;
}

void Task4DContactNormalTask::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	J_task = J_tmp.bottomRows(4);

	// Compute the top row to be the Jacobian of the distance between the contact point and the plane
	computeError();
	J_task.row(0) = (1.0/(closest_point_on_plane_-cur_pos_).norm())*(closest_point_on_plane_ - cur_pos_).transpose()*J_tmp.topRows(3);



}
void Task4DContactNormalTask::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	Jdot_task = Jdot_tmp.bottomRows(4);

	// Compute the top row to be the Jacobian dot of the distance between the contact point and the plane
	computeError();
	Jdot_task.row(0) = (1.0/(closest_point_on_plane_-cur_pos_).norm())*(closest_point_on_plane_ - cur_pos_).transpose()*Jdot_tmp.topRows(3);
}


void Task4DContactNormalTask::computeError(){
	// Get current position of the end-effector
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);
	
	// Compute signed perpendicular distance to the plane.
	// v = end_effector - plane_center
	// distance = v \cdot normal_vec_hat
	dist_to_plane = (cur_pos_ - normal_vec_tail_).dot(normal_vec_hat_);

	// Compute the desired position, which is the point on the plane closest to the end effector link
	des_pos = cur_pos_ - normal_vec_hat_*dist_to_plane;
	closest_point_on_plane_ = des_pos;

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
	// math_utils::printQuat(quat_omega_);
	des_quat = quat_omega_*quat_current_;

	// Compute Errors
	// Compute Linear Error
	error_[0] = kp_task_gain_*(des_pos - cur_pos_).norm();

	// Compute Quaternion Error
	error_.tail(3) = kp_task_gain_*(omega_.angle()*omega_.axis());	
}

// Computes the error for a given reference
void Task4DContactNormalTask::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

