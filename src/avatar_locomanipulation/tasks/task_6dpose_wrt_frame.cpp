#include <avatar_locomanipulation/tasks/task_6dpose_wrt_frame.hpp>

Task6DPosewrtFrame::Task6DPosewrtFrame(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name, const std::string & input_wrt_frame_name):
	Task6DPose(input_model, input_frame_name){
	des_pos.setZero();
	des_quat.setIdentity();

	frame_pos.setZero();
	frame_quat.setIdentity();
	wrt_frame_name = input_wrt_frame_name;

	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	J_wrt = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_wrt = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	std::cout << "[Task6DPosewrtFrame] for frame " << frame_name << " w.r.t. frame " << wrt_frame_name << " Constructed" << std::endl;
}

Task6DPosewrtFrame::~Task6DPosewrtFrame(){
	// std::cout << "[Task6DPosewrtFrame] for frame " << frame_name << " Destroyed" << std::endl;
}

void Task6DPosewrtFrame::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	robot_model->get6DTaskJacobian(wrt_frame_name, J_wrt);
	J_task = J_tmp - J_wrt;
}
void Task6DPosewrtFrame::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	robot_model->get6DTaskJacobianDot(wrt_frame_name, Jdot_wrt);
	Jdot_task = Jdot_tmp - Jdot_wrt;
}

void Task6DPosewrtFrame::computeError(){
	// Get Frame Position and Orientation
	robot_model->getFrameWorldPose(wrt_frame_name, frame_pos, frame_quat);
	// Get 6D pose of this link
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);

	// Compute desired values:
	des_pos = frame_pos + frame_quat.toRotationMatrix()*vec_ref_;
	des_quat = frame_quat*quat_ref_;

	// std::cout << "des_pos = " << des_pos << std::endl;
	// std::cout << "des_quat = ";
	// math_utils::printQuat(des_quat);

	Eigen::AngleAxisd des_aa(des_quat);
	Eigen::AngleAxisd cur_aa(quat_current_);

	// Compute Errors
	// Compute Linear Error
	error_.head(3) = kp_task_gain_*(des_pos - cur_pos_);
	// Compute Quaternion Error
	math_utils::compute_quat_error(des_quat, quat_current_, quat_error_);
	error_.tail(3) = kp_task_gain_*(des_aa.axis()*des_aa.angle() - cur_aa.axis()*cur_aa.angle()); //kp_task_gain_*(quat_error_);	

	std::cout << "Frame:" << frame_name << " wrt frame " << wrt_frame_name << " error = " << error_.transpose() << std::endl;
}
