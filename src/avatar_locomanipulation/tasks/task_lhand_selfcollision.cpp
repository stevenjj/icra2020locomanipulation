#include <avatar_locomanipulation/tasks/task_lhand_selfcollision.hpp>

TaskLhandSelfCollision::TaskLhandSelfCollision(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name, std::shared_ptr<CollisionEnvironment> & collision){
	robot_model = input_model;
	task_dim = 3;
	task_name = input_frame_name;
	frame_name = input_frame_name;

	error_ = Eigen::VectorXd::Zero(task_dim);
	vec_ref_ = Eigen::VectorXd::Zero(3);
	quat_ref_.setIdentity();

	collision_env = collision;

	std::cout << "[Task Lhand Self Collision] for frame " << frame_name << " Constructed" << std::endl;
}

TaskLhandSelfCollision::~TaskLhandSelfCollision(){
	std::cout << "[Task Lhand Self Collision] for frame " << frame_name << " Destroyed" << std::endl;
}

void TaskLhandSelfCollision::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_task);
}
void TaskLhandSelfCollision::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_task);
}



// Set Task References
void TaskLhandSelfCollision::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}

void TaskLhandSelfCollision::setReference(const Eigen::Quaterniond & quat_ref_in){
	quat_ref_ = quat_ref_in;
}


void TaskLhandSelfCollision::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	vec_ref_ = vec_ref_in;
	quat_ref_ = quat_ref_in;
}


// Get Task References
void TaskLhandSelfCollision::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}

void TaskLhandSelfCollision::getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	vec_ref_out = vec_ref_;
	quat_ref_out = quat_ref_;
}

void TaskLhandSelfCollision::getReference(Eigen::Quaterniond & quat_ref_out){
	quat_ref_out = quat_ref_;
}

void TaskLhandSelfCollision::computeError(){

	collision_env->valkyrie->updateFullKinematics(collision_env->valkyrie->q_current);

	// Define the map for val_world_positions
 	std::map<std::string, Eigen::Vector3d> world_positions = collision_env->find_world_positions_subset();

 	collision_env->self_directed_vectors.clear();
	collision_env->build_directed_vector_to_lhand(world_positions);

	std::vector<Eigen::Vector3d> dxs = collision_env->self_collision_dx();

	Eigen::Vector3d dx;
	dx = dxs[0];

	for(int i=1; i<dxs.size(); ++i){
		dx += dxs[i];
	}

	error_[0] = dx[0];
	error_[1] = dx[1];
	error_[2] = dx[2];
}

// Computes the error for a given reference
void TaskLhandSelfCollision::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

// Sets the task error manually
void TaskLhandSelfCollision::setError(const Eigen::VectorXd & error_in){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}