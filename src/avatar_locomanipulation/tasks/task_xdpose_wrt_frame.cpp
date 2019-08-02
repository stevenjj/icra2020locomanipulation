#include <avatar_locomanipulation/tasks/task_xdpose_wrt_frame.hpp>

TaskXDPosewrtFrame::TaskXDPosewrtFrame(std::shared_ptr<RobotModel> & input_model, 
					   std::vector<int> input_task_dimensions,
					   const std::string & input_frame_name, 
					   const std::string & input_wrt_frame_name):Task6DPose(input_model, input_frame_name){
	if (task_dimensions.size() > 6){
		std::cout << "[TaskXDPosewrtFrame] Error. Task Dimensions must be at most 6." << std::endl;
		throw;
	}
	task_dimensions = input_task_dimensions;

	robot_model = input_model;
	task_dim = task_dimensions.size();
	task_name = input_frame_name;
	frame_name = input_frame_name;

	error_tmp = Eigen::VectorXd::Zero(6);
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

	// We will only compute Jacobians and errors for the task dimension specified
	error_ = Eigen::VectorXd::Zero(task_dim);
	J_out = Eigen::MatrixXd::Zero(task_dim, robot_model->getDimQdot());
	Jdot_out = Eigen::MatrixXd::Zero(task_dim, robot_model->getDimQdot());

	// std::cout << "[TaskXDPosewrtFrame] for frame " << frame_name << " w.r.t. frame " << wrt_frame_name << " Constructed" << std::endl;
}

TaskXDPosewrtFrame::~TaskXDPosewrtFrame(){
	// std::cout << "[TaskXDPosewrtFrame] for frame " << frame_name << " Destroyed" << std::endl;
}

void TaskXDPosewrtFrame::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	robot_model->get6DTaskJacobian(wrt_frame_name, J_wrt);

	for(int i = 0 ; i < task_dimensions.size(); i++){
		J_out.row(i) = J_tmp.row(task_dimensions[i])- J_wrt.row(task_dimensions[i]); 
	}
	J_task = J_out;

}
void TaskXDPosewrtFrame::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	robot_model->get6DTaskJacobianDot(wrt_frame_name, Jdot_wrt);

	for(int i = 0 ; i < task_dimensions.size(); i++){
		Jdot_out.row(i) = Jdot_tmp.row(task_dimensions[i])- Jdot_wrt.row(task_dimensions[i]); 
	}
	Jdot_task = Jdot_out;

}

void TaskXDPosewrtFrame::computeError(){
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
	error_tmp.head(3) = kp_task_gain_*(des_pos - cur_pos_);
	// Compute Quaternion Error
	math_utils::compute_quat_error(des_quat, quat_current_, quat_error_);
	error_tmp.tail(3) = kp_task_gain_*(des_aa.axis()*des_aa.angle() - cur_aa.axis()*cur_aa.angle()); //kp_task_gain_*(quat_error_);	

	// Output the error
	for(int i = 0 ; i < task_dimensions.size(); i++){
		error_[i] = error_tmp[task_dimensions[i]];
	}

}
