#include <avatar_locomanipulation/tasks/task_6dpose.hpp>

Task6DPose::Task6DPose(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name){
	robot_model = input_model;
	task_dim = 6;
	task_name = input_frame_name;
	frame_name = input_frame_name;
	std::cout << "[Task 6D Pose] for frame " << frame_name << " Constructed" << std::endl;
}

Task6DPose::~Task6DPose(){
	std::cout << "[Task 6D Pose] for frame " << frame_name << " Destroyed" << std::endl;
}

void Task6DPose::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_task);
}
void Task6DPose::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_task);
}
