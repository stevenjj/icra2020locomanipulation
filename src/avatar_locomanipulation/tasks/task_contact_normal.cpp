#include <avatar_locomanipulation/tasks/task_contact_normal.hpp>

TaskContactNormalTask::TaskContactNormalTask(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name, 
					 						     const Eigen::Vector3d & normal_vec, const Eigen::Vector3d normal_vec_tail){
	robot_model = input_model;
	task_dim = 3;
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
	ones_ = Eigen::Vector3d(1.0, 1.0, 1.0);

	des_pos.setZero();
	des_quat.setIdentity();
	std::cout << "[TaskContactNormalTask] for frame " << frame_name << " Constructed" << std::endl;

	error_ = Eigen::VectorXd::Zero(task_dim);

	std::cout << "[TaskContactNormalTask] for frame " << frame_name << " Constructed" << std::endl;
}

TaskContactNormalTask::~TaskContactNormalTask(){
	// std::cout << "[TaskContactNormalTask] for frame " << frame_name << " Destroyed" << std::endl;
}

void TaskContactNormalTask::getTaskJacobian(Eigen::MatrixXd & J_task){
	// Get the local frame Jacobian (Body Jacobian)
	robot_model->get6DTaskJacobianLocal(frame_name, J_tmp);
	J_task = J_tmp.bottomRows(3); 

	// Compute the top row to be the Jacobian of the distance between the contact point and the plane
	computeError();

	if (fabs(dist_to_plane) > 1e-6){
		J_task.row(2) = (1.0/ (R_frame_ori_.transpose()*(closest_point_on_plane_-cur_pos_)).norm()) * (R_frame_ori_*(closest_point_on_plane_ - cur_pos_)).transpose()*J_tmp.topRows(3);
	}else{
		J_task.row(2) = Eigen::MatrixXd::Zero(1, robot_model->getDimQdot());		
	}

}
void TaskContactNormalTask::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	// Get the local frame Jacobian dot (Body Jacobian Dot)
	robot_model->get6DTaskJacobianDotLocal(frame_name, Jdot_tmp);
	Jdot_task = Jdot_tmp.bottomRows(3);

	// Compute the top row to be the Jacobian dot of the distance between the contact point and the plane
	computeError();
	if (fabs(dist_to_plane) > 1e-6){
		Jdot_task.row(2) = (1.0/(R_frame_ori_.transpose()*(closest_point_on_plane_-cur_pos_)).norm())*(R_frame_ori_*(closest_point_on_plane_ - cur_pos_)).transpose()*Jdot_tmp.topRows(3);
	}else{
		Jdot_task.row(2) = Eigen::MatrixXd::Zero(1, robot_model->getDimQdot());		
	}

}


void TaskContactNormalTask::computeError(){
	// Get current position of the end-effector w.r.t world
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);

	// Get z hat vector of the end effector frame
	R_frame_ori_ = quat_current_.toRotationMatrix();
	z_hat_ = R_frame_ori_.col(2); // Third column of the rotation matrix	

	// Compute signed perpendicular distance to the plane w.r.t world.
	// v = end_effector - plane_center
	// distance = v \cdot normal_vec_hat
	dist_to_plane = (cur_pos_ - normal_vec_tail_).dot(normal_vec_hat_);

	// Compute the desired position, which is the point on the plane closest to the end effector link
	des_pos = cur_pos_ - normal_vec_hat_*dist_to_plane;
	closest_point_on_plane_ = des_pos;

	// Compute orientation error (zhat) to normal vector zhat w.r.t world .
	theta_angle_ = acos(z_hat_.dot(normal_vec_hat_));
	omega_hat_ =  z_hat_.cross(normal_vec_hat_);
	omega_hat_.normalize();
	omega_.angle() = theta_angle_;
	omega_.axis() = omega_hat_;
	// std::cout << "Axis " << omega_.axis().transpose() << ", Angle  " << omega_.angle() << std::endl;  

	// Find desired quaternion w.r.t world
	quat_omega_ = omega_;
	des_quat = quat_omega_*quat_current_;

	// Compute Errors w.r.t the local frame of the end effector
	// Compute Quaternion Error
	error_.head(2) = kp_task_gain_*(  R_frame_ori_.transpose()*omega_.axis()*omega_.angle()).head(2);
	// Compute Linear Error
	error_[2] = kp_task_gain_*( R_frame_ori_.transpose()*(des_pos - cur_pos_) ) .norm();

}

// Computes the error for a given reference
void TaskContactNormalTask::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

