#include <avatar_locomanipulation/tasks/task_6dpose_wrt_midfeet.hpp>

Task6DPosewrtMidFeet::Task6DPosewrtMidFeet(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name): 
	Task6DPose(input_model, input_frame_name){
	des_pos.setZero();
	des_quat.setIdentity();
	left_foot_frame = "leftCOP_Frame";
	right_foot_frame = "rightCOP_Frame";
	right_foot.robot_side = RIGHT_FOOTSTEP;
	midfeet.robot_side = MID_FOOTSTEP;

	// task_dim = 3;
	// error_ = Eigen::VectorXd::Zero(task_dim);
	// vec_ref_ = Eigen::VectorXd::Zero(3);
	// quat_ref_.setIdentity();

	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	J_lf = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	J_rf = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());	

	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_lf = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_rf = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	std::cout << "[Task6DPosewrtMidFeet] for frame " << frame_name << " Constructed" << std::endl;
}

Task6DPosewrtMidFeet::~Task6DPosewrtMidFeet(){
	// std::cout << "[Task6DPosewrtMidFeet] for frame " << frame_name << " Destroyed" << std::endl;
}

void Task6DPosewrtMidFeet::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	robot_model->get6DTaskJacobian(left_foot_frame, J_lf);
	robot_model->get6DTaskJacobian(right_foot_frame, J_rf);
	// 2nd term is the midfeet frame Jacobian
	J_task = J_tmp - 0.5*(J_lf + J_rf);
}
void Task6DPosewrtMidFeet::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	robot_model->get6DTaskJacobianDot(left_foot_frame, Jdot_lf);
	robot_model->get6DTaskJacobianDot(right_foot_frame, Jdot_rf);
	Jdot_task = Jdot_tmp - 0.5*(Jdot_lf + Jdot_rf);
}

void Task6DPosewrtMidFeet::computeError(){
	// Get Left Foot Pos/Ori. Compute Rotation Matrix
	robot_model->getFrameWorldPose(left_foot_frame, left_foot.position, left_foot.orientation);
	left_foot.R_ori = left_foot.orientation.toRotationMatrix();	
	// Get Right Foot Pos/Ori. Compute Rotation Matrix
	robot_model->getFrameWorldPose(right_foot_frame, right_foot.position, right_foot.orientation);
	right_foot.R_ori = right_foot.orientation.toRotationMatrix();	
	// Get 6D pose of this link.
	robot_model->getFrameWorldPose(frame_name, cur_pos_, quat_current_);

	// Compute Midfeet
	midfeet.computeMidfeet(left_foot, right_foot, midfeet);

	// std::cout << "left_foot.position" << left_foot.position.transpose() << std::endl;
	// std::cout << "right_foot.position" << right_foot.position.transpose() << std::endl;
	// std::cout << "midfeet.position" << midfeet.position.transpose() << std::endl;
	// std::cout << "cur_pos_" << cur_pos_.transpose() << std::endl;

	// Compute desired values:
	des_pos = midfeet.position + midfeet.R_ori*vec_ref_;
	des_quat = midfeet.orientation*quat_ref_;

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

	// std::cout << "pelvis wrt midfeet error_ = " << error_.transpose() << std::endl;
}
