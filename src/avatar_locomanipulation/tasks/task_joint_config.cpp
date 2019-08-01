#include <avatar_locomanipulation/tasks/task_joint_config.hpp>

TaskJointConfig::TaskJointConfig(std::shared_ptr<RobotModel> & input_model, const std::vector<std::string> & joint_names){
	robot_model = input_model;
	task_dim = joint_names.size();
	J_config = Eigen::MatrixXd::Zero(task_dim, robot_model->getDimQdot());
	Jdot_config = Eigen::MatrixXd::Zero(task_dim, robot_model->getDimQdot());
	cur_joint_pos = Eigen::VectorXd::Zero(task_dim);
	joint_names_ = joint_names;

	task_name = "configs ";
	frame_name = "no_frames";
	int joint_index = 0;
	for(int i = 0; i < joint_names.size(); i++){
		task_name = task_name + " " + joint_names[i];
		joint_index = 6 + robot_model->getJointIndexNoFloatingJoints(joint_names[i]);
		J_config(i, joint_index) = 1;
	}
	std::cout << "[Task Joint Config] for joints " << task_name << " Constructed" << std::endl;
}

TaskJointConfig::~TaskJointConfig(){
	// std::cout << "[Task Joint Config] for joints " << task_name << " Destroyed" << std::endl;
}

void TaskJointConfig::getTaskJacobian(Eigen::MatrixXd & J_task){
	J_task = J_config;
}
void TaskJointConfig::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	Jdot_task = Jdot_config;
}

std::vector<std::string> TaskJointConfig::getJointNames(){
	return joint_names_;
}

void TaskJointConfig::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}
void TaskJointConfig::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}
void TaskJointConfig::computeError(){
	for(int i = 0; i < joint_names_.size(); i++){
		cur_joint_pos[i] = robot_model->q_current[robot_model->getJointIndex(joint_names_[i])];
	}
	error_ = kp_task_gain_*(vec_ref_ - cur_joint_pos);
}
void TaskJointConfig::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}
