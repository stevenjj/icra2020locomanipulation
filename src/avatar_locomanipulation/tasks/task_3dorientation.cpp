#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>

Task3DOrientation::Task3DOrientation(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name){
	robot_model = input_model;
	task_dim = 3;
	task_name = input_frame_name;
	frame_name = input_frame_name;
	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	std::cout << "[Task 3D Orientation] for frame " << frame_name << " Constructed" << std::endl;
}

Task3DOrientation::~Task3DOrientation(){
	std::cout << "[Task 3D Orientation] for frame " << frame_name << " Destroyed" << std::endl;
}

void Task3DOrientation::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	J_task = J_tmp.bottomRows(3);
}
void Task3DOrientation::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	Jdot_task = Jdot_tmp.bottomRows(3);
}


void Task3DOrientation::setReference(const Eigen::Quaterniond & quat_ref_in){
}

void Task3DOrientation::getReference(Eigen::Quaterniond & quat_ref_out){
}
void Task3DOrientation::computeError(){
}
void Task3DOrientation::getError(Eigen::VectorXd & error_out, bool compute){
}

