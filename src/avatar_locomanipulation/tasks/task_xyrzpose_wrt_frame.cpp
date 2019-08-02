#include <avatar_locomanipulation/tasks/task_xyrzpose_wrt_frame.hpp>

TaskXYRZPosewrtFrame::TaskXYRZPosewrtFrame(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name, const std::string & input_wrt_frame_name):
	Task6DPose(input_model, input_frame_name){

	task_dim = 3;

	error_ = Eigen::VectorXd::Zero(task_dim);
	vec_ref_ = Eigen::VectorXd::Zero(3);
	quat_ref_.setIdentity();

	des_pos.setZero();
	des_quat.setIdentity();

	frame_pos.setZero();
	frame_quat.setIdentity();
	wrt_frame_name = input_wrt_frame_name;

	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	J_wrt = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_wrt = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	std::cout << "[TaskXYRZPosewrtFrame] for frame " << frame_name << " w.r.t. frame " << wrt_frame_name << " Constructed" << std::endl;
}

TaskXYRZPosewrtFrame::~TaskXYRZPosewrtFrame(){
	// std::cout << "[TaskXYRZPosewrtFrame] for frame " << frame_name << " Destroyed" << std::endl;
}

void TaskXYRZPosewrtFrame::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	robot_model->get6DTaskJacobian(wrt_frame_name, J_wrt);
	J_task = J_tmp.topRows(3) - J_wrt.topRows(3);
	J_task.row(2) = J_tmp.row(5) - J_wrt.row(5);


}
void TaskXYRZPosewrtFrame::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	robot_model->get6DTaskJacobianDot(wrt_frame_name, Jdot_wrt);
	Jdot_task = Jdot_tmp.topRows(3) - Jdot_wrt.topRows(3);
	Jdot_task.row(2) = Jdot_tmp.row(5) - Jdot_wrt.row(5);

}

void TaskXYRZPosewrtFrame::computeError(){
	// Get Frame Position and Orientation
	robot_model->getFrameWorldPose(wrt_frame_name, frame_pos, frame_quat);
	// Get 6D pose of this link
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);

	// Compute desired values:
	des_pos = frame_pos + frame_quat.toRotationMatrix()*vec_ref_;
	des_quat = frame_quat*quat_ref_;

	Eigen::AngleAxisd des_aa(des_quat);
	Eigen::AngleAxisd cur_aa(quat_current_);

	// Compute Errors
	// Compute Linear Error
	error_.head(2) = kp_task_gain_*(des_pos.head(2) - cur_pos_.head(2));
	// Compute Quaternion Error
	math_utils::compute_quat_error(des_quat, quat_current_, quat_error_);
	error_[2] = kp_task_gain_*(des_aa.axis()*des_aa.angle() - cur_aa.axis()*cur_aa.angle())[2];


}
