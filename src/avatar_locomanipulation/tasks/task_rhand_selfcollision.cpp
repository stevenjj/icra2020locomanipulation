#include <avatar_locomanipulation/tasks/task_rhand_selfcollision.hpp>

TaskRhandSelfCollision::TaskRhandSelfCollision(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name, std::shared_ptr<CollisionEnvironment> & collision){
	robot_model = input_model;
	task_dim = 3;
	task_name = input_frame_name;
	frame_name = input_frame_name;

	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	error_ = Eigen::VectorXd::Zero(task_dim);
	vec_ref_ = Eigen::VectorXd::Zero(3);
	quat_ref_.setIdentity();

	collision_env = collision;

	std::cout << "[Task Rhand Self Collision] for frame " << frame_name << " Constructed" << std::endl;
}

TaskRhandSelfCollision::~TaskRhandSelfCollision(){
	std::cout << "[Task Rhand Self Collision] for frame " << frame_name << " Destroyed" << std::endl;
}

void TaskRhandSelfCollision::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	J_task = J_tmp.topRows(3);	
}
void TaskRhandSelfCollision::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	Jdot_task = Jdot_tmp.topRows(3);
}

// Set Task References
void TaskRhandSelfCollision::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}

void TaskRhandSelfCollision::setReference(const Eigen::Quaterniond & quat_ref_in){
	quat_ref_ = quat_ref_in;
}


void TaskRhandSelfCollision::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	vec_ref_ = vec_ref_in;
	quat_ref_ = quat_ref_in;
}


// Get Task References
void TaskRhandSelfCollision::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}

void TaskRhandSelfCollision::getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	vec_ref_out = vec_ref_;
	quat_ref_out = quat_ref_;
}

void TaskRhandSelfCollision::getReference(Eigen::Quaterniond & quat_ref_out){
	quat_ref_out = quat_ref_;
}

void TaskRhandSelfCollision::computeError(){

	// collision_env->valkyrie->updateFullKinematics(collision_env->valkyrie->q_current);

	// Define the map for val_world_positions
 	std::map<std::string, Eigen::Vector3d> world_positions = collision_env->find_world_positions_subset();

 	collision_env->self_directed_vectors.clear();
	collision_env->build_directed_vector_to_rhand(world_positions);

	// for (int j=0; j<collision_env->self_directed_vectors.size(); ++j){
	// 	std::cout << "collision_env->self_directed_vectors[j].to: " << collision_env->self_directed_vectors[j].to << std::endl;
	// 	std::cout << "collision_env->self_directed_vectors[j].from: " << collision_env->self_directed_vectors[j].from << std::endl;
	// 	std::cout << "collision_env->self_directed_vectors[j].magnitude: " << collision_env->self_directed_vectors[j].magnitude << std::endl;
	// 	std::cout << "collision_env->self_directed_vectors[j].direction: " << collision_env->self_directed_vectors[j].direction << std::endl;
	// }
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
void TaskRhandSelfCollision::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

// Sets the task error manually
void TaskRhandSelfCollision::setError(const Eigen::VectorXd & error_in){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}