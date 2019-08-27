#include <avatar_locomanipulation/tasks/task_objectcollision.hpp>

TaskObjectCollision::TaskObjectCollision(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name, std::shared_ptr<CollisionEnvironment> & collision, const std::string & link_name_in){
	robot_model = input_model;
	task_dim = 1;
	task_name = input_frame_name;
	frame_name = input_frame_name;

	link_name = link_name_in;

	J_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());
	Jdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	error_ = Eigen::VectorXd::Zero(task_dim);
	vec_ref_ = Eigen::VectorXd::Zero(3);
	quat_ref_.setIdentity();

	collision_env = collision;
	eta = collision_env->eta;

	std::cout << "[Task " << link_name << " Object Collision] for frame " << frame_name << " Constructed" << std::endl;
}

TaskObjectCollision::~TaskObjectCollision(){
	std::cout << "[Task " << link_name << " Object Collision] for frame " << frame_name << " Destroyed" << std::endl;
}

void TaskObjectCollision::getTaskJacobian(Eigen::MatrixXd & J_task){
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	J_task = Eigen::MatrixXd::Zero(1, robot_model->getDimQdot());

	// If the links are in collision then we want higher safety distance
	if(collision_env->directed_vectors[collision_env->closest].using_worldFramePose){
		// Set this to J_task
		J_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(0.2)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (- J_tmp.topRows(3));
		return;
	} 
	// Else we want lower safety distance
	else{
		// If magnitude inside safety distance
		if(collision_env->directed_vectors[collision_env->closest].magnitude < 0.075){
  			// Add this to J_task
  			J_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(0.075)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (- J_tmp.topRows(3));
  		}	
	} 	
}
void TaskObjectCollision::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){
	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	Jdot_task = Eigen::MatrixXd::Zero(1, robot_model->getDimQdot());

	
	std:: cout << "collision_env->directed_vectors[collision_env->closest].from: " << collision_env->directed_vectors[collision_env->closest].from << std::endl;
	// If the links are in collision then we want higher safety distance
	if(collision_env->directed_vectors[collision_env->closest].using_worldFramePose){
		// If magnitude inside safety distance
  		if(collision_env->directed_vectors[collision_env->closest].magnitude < collision_env->safety_dist_collision){
  			// Add this to Jdot_task
  			Jdot_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(collision_env->safety_dist_collision)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (- Jdot_tmp.topRows(3));
  		}
	} 
	// Else we want lower safety distance
	else{
		// If magnitude inside safety distance
		if(collision_env->directed_vectors[collision_env->closest].magnitude < collision_env->safety_dist_normal){
  			// Add this to J_task
  			Jdot_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(collision_env->safety_dist_normal)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (-Jdot_tmp.topRows(3));
  		}	
	} 
}

// Set Task References
void TaskObjectCollision::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}

void TaskObjectCollision::setReference(const Eigen::Quaterniond & quat_ref_in){
	quat_ref_ = quat_ref_in;
}


void TaskObjectCollision::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	vec_ref_ = vec_ref_in;
	quat_ref_ = quat_ref_in;
}


// Get Task References
void TaskObjectCollision::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}

void TaskObjectCollision::getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	vec_ref_out = vec_ref_;
	quat_ref_out = quat_ref_;
}

void TaskObjectCollision::getReference(Eigen::Quaterniond & quat_ref_out){
	quat_ref_out = quat_ref_;
}

void TaskObjectCollision::computeError(){
	Eigen::VectorXd q = robot_model->q_current;
	
 	collision_env->directed_vectors.clear();

 	collision_env->build_object_directed_vectors(frame_name, robot_model->q_current);

 	double V = collision_env->get_collision_potential();

	error_[0] = kp_task_gain_*V;
}

// Computes the error for a given reference
void TaskObjectCollision::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

// Sets the task error manually
void TaskObjectCollision::setError(const Eigen::VectorXd & error_in){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}