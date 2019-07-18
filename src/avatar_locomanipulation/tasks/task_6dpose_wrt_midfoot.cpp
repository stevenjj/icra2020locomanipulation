#include <avatar_locomanipulation/tasks/task_6dpose_wrt_midfeet.hpp>

Task6DPosewrtMidFeet::Task6DPosewrtMidFeet(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name): 
	Task6DPose(input_model, input_frame_name){
	des_pos.setZero();
	des_quat.setIdentity();
	left_foot_frame = "leftCOP_Frame";
	right_foot_frame = "rightCOP_Frame";
	right_foot.robot_side = RIGHT_FOOTSTEP;
	midfeet.robot_side = MID_FOOTSTEP;
	std::cout << "[Task6DPosewrtMidFeet] for frame " << frame_name << " Constructed" << std::endl;
}

Task6DPosewrtMidFeet::~Task6DPosewrtMidFeet(){
	// std::cout << "[Task6DPosewrtMidFeet] for frame " << frame_name << " Destroyed" << std::endl;
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

	// Compute Errors
	// Compute Linear Error
	error_.head(3) = kp_task_gain_*(des_pos - cur_pos_);
	// Compute Quaternion Error
	math_utils::compute_quat_error(des_quat, quat_current_, quat_error_);
	error_.tail(3) = kp_task_gain_*(quat_error_);	

	std::cout << "pelvis wrt midfeet error_ = " << error_.transpose() << std::endl;
}
