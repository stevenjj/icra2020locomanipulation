#include <avatar_locomanipulation/tasks/task_pointcollision.hpp>

TaskPointCollision::TaskPointCollision(const std::string task_name_in, std::shared_ptr<RobotModel> & input_model, std::shared_ptr<CollisionEnvironment> & collision, const std::vector<Eigen::Vector3d> & point_list_in){
	robot_model = input_model;
	task_dim = 1;
	task_name = task_name_in;

	points_to_avoid = point_list_in;

	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	error_ = Eigen::VectorXd::Zero(task_dim);
	vec_ref_ = Eigen::VectorXd::Zero(3);
	quat_ref_.setIdentity();

	collision_env = collision;
	eta = collision_env->eta;

	std::cout << "[Task " << task_name << " Self Collision] Constructed" << std::endl;
}

TaskPointCollision::~TaskPointCollision(){
	std::cout << "[Task " << task_name << " Self Collision] Destroyed" << std::endl;
}

void TaskPointCollision::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(nearest_robot_frame, J_tmp);
	J_task = Eigen::MatrixXd::Zero(1, robot_model->getDimQdot());

	// If the links are in collision then we want higher safety distance
	if(collision_env->directed_vectors[collision_env->closest].using_worldFramePose){
		// Set this to J_task
		J_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(collision_env->safety_dist_collision)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (J_tmp.topRows(3));
		return;
	} 
	// Else we want lower safety distance
	else{
		// If magnitude inside safety distance
		if(collision_env->directed_vectors[collision_env->closest].magnitude < collision_env->safety_dist_normal){
  			// Add this to J_task
  			J_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(collision_env->safety_dist_normal)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (J_tmp.topRows(3));
  		}	
	} 	
}
void TaskPointCollision::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(nearest_robot_frame, Jdot_tmp);
	Jdot_task = Eigen::MatrixXd::Zero(1, robot_model->getDimQdot());

	
	std:: cout << "collision_env->directed_vectors[collision_env->closest].from: " << collision_env->directed_vectors[collision_env->closest].from << std::endl;
	// If the links are in collision then we want higher safety distance
	if(collision_env->directed_vectors[collision_env->closest].using_worldFramePose){
		// If magnitude inside safety distance
  		if(collision_env->directed_vectors[collision_env->closest].magnitude < collision_env->safety_dist_collision){
  			// Add this to Jdot_task
  			Jdot_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(collision_env->safety_dist_collision)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (Jdot_tmp.topRows(3));
  		}
	} 
	// Else we want lower safety distance
	else{
		// If magnitude inside safety distance
		if(collision_env->directed_vectors[collision_env->closest].magnitude < collision_env->safety_dist_normal){
  			// Add this to J_task
  			Jdot_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(collision_env->safety_dist_normal)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (Jdot_tmp.topRows(3));
  		}	
	} 
}

// Set Task References
void TaskPointCollision::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}

void TaskPointCollision::setReference(const Eigen::Quaterniond & quat_ref_in){
	quat_ref_ = quat_ref_in;
}


void TaskPointCollision::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	vec_ref_ = vec_ref_in;
	quat_ref_ = quat_ref_in;
}


// Get Task References
void TaskPointCollision::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}

void TaskPointCollision::getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	vec_ref_out = vec_ref_;
	quat_ref_out = quat_ref_;
}

void TaskPointCollision::getReference(Eigen::Quaterniond & quat_ref_out){
	quat_ref_out = quat_ref_;
}

void TaskPointCollision::computeError(){
	Eigen::VectorXd q = robot_model->q_current;
	
 	collision_env->directed_vectors.clear();
 	// Fills dvectors from each point in list to important robot frames
 	collision_env->build_point_list_directed_vectors(points_to_avoid, robot_model->q_current);
	// Sorts through the dvectors and gets potential for the shortes pair
 	double V = collision_env->get_collision_potential();
 	// Name of the robot frame that is in the nearest pair
 	nearest_robot_frame = collision_env->directed_vectors[collision_env->closest].to;

	error_[0] = kp_task_gain_*V;
	std::cout << "error_[0]: " << error_[0] << std::endl;
}

// Computes the error for a given reference
void TaskPointCollision::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

// Sets the task error manually
void TaskPointCollision::setError(const Eigen::VectorXd & error_in){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}