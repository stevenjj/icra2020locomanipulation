#include <avatar_locomanipulation/tasks/task_selfcollision.hpp>

TaskSelfCollision::TaskSelfCollision(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name, std::shared_ptr<CollisionEnvironment> & collision, const std::string & link_name_in){
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

	std::cout << "[Task " << link_name << " Self Collision] for frame " << frame_name << " Constructed" << std::endl;
}

TaskSelfCollision::~TaskSelfCollision(){
	std::cout << "[Task " << link_name << " Self Collision] for frame " << frame_name << " Destroyed" << std::endl;
}

void TaskSelfCollision::getTaskJacobian(Eigen::MatrixXd & J_task){

	Eigen::MatrixXd Jp_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());

	std::cout << "st1\n";
	robot_model->get6DTaskJacobian(frame_name, J_tmp);
	std::cout << "st2\n";
	J_task = Eigen::MatrixXd::Zero(1, robot_model->getDimQdot());

	std::cout << "collision_env->directed_vectors[collision_env->closest].from: " << collision_env->directed_vectors[collision_env->closest].from << std::endl;
	robot_model->get6DTaskJacobian(collision_env->directed_vectors[collision_env->closest].from, Jp_tmp);

	// If the links are in collision then we want higher safety distance
	if(collision_env->directed_vectors[collision_env->closest].using_worldFramePose){
		// Add this to J_task
		std::cout << "Task1" << std::endl;
		J_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(0.2)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (Jp_tmp.topRows(3) - J_tmp.topRows(3));
		return;
	} 
	// Else we want lower safety distance
	else{
		// If magnitude inside safety distance
		if(collision_env->directed_vectors[collision_env->closest].magnitude < 0.075){
  			// Add this to J_task
  			std::cout << "Task2" << std::endl;
  			J_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(0.075)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (Jp_tmp.topRows(3) - J_tmp.topRows(3));
  		}	
	} 


}
void TaskSelfCollision::getTaskJacobianDot(Eigen::MatrixXd & Jdot_task){

	Eigen::MatrixXd Jpdot_tmp = Eigen::MatrixXd::Zero(6, robot_model->getDimQdot());


	robot_model->get6DTaskJacobianDot(frame_name, Jdot_tmp);
	Jdot_task = Eigen::MatrixXd::Zero(1, robot_model->getDimQdot());

	
	std:: cout << "collision_env->directed_vectors[collision_env->closest].from: " << collision_env->directed_vectors[collision_env->closest].from << std::endl;
	robot_model->get6DTaskJacobianDot(collision_env->directed_vectors[collision_env->closest].from, Jpdot_tmp);
	// If the links are in collision then we want higher safety distance
	if(collision_env->directed_vectors[collision_env->closest].using_worldFramePose){
		// If magnitude inside safety distance
  		if(collision_env->directed_vectors[collision_env->closest].magnitude < collision_env->safety_dist_collision){
  			// Add this to Jdot_task
  			Jdot_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(collision_env->safety_dist_collision)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (Jpdot_tmp.topRows(3) - Jdot_tmp.topRows(3));
  		}
	} 
	// Else we want lower safety distance
	else{
		// If magnitude inside safety distance
		if(collision_env->directed_vectors[collision_env->closest].magnitude < collision_env->safety_dist_normal){
  			// Add this to J_task
  			Jdot_task = eta * ( (1/(collision_env->directed_vectors[collision_env->closest].magnitude)) - (1/(collision_env->safety_dist_normal)) ) * ((-1)/(std::pow((collision_env->directed_vectors[collision_env->closest].magnitude),2))) * (1/((collision_env->directed_vectors[collision_env->closest].magnitude))) * ((collision_env->directed_vectors[collision_env->closest].magnitude)*(collision_env->directed_vectors[collision_env->closest].direction).transpose()) * (Jpdot_tmp.topRows(3) - Jdot_tmp.topRows(3));
  		}	
	} 

}

// Set Task References
void TaskSelfCollision::setReference(const Eigen::VectorXd & vec_ref_in){
	vec_ref_ = vec_ref_in;
}

void TaskSelfCollision::setReference(const Eigen::Quaterniond & quat_ref_in){
	quat_ref_ = quat_ref_in;
}


void TaskSelfCollision::setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in){
	vec_ref_ = vec_ref_in;
	quat_ref_ = quat_ref_in;
}


// Get Task References
void TaskSelfCollision::getReference(Eigen::VectorXd & vec_ref_out){
	vec_ref_out = vec_ref_;
}

void TaskSelfCollision::getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out){
	vec_ref_out = vec_ref_;
	quat_ref_out = quat_ref_;
}

void TaskSelfCollision::getReference(Eigen::Quaterniond & quat_ref_out){
	quat_ref_out = quat_ref_;
}

void TaskSelfCollision::computeError(){

 	collision_env->directed_vectors.clear();

 	collision_env->build_self_directed_vectors(frame_name, robot_model->q_current);

 	double V = collision_env->get_collision_potential();

	error_[0] = kp_task_gain_*V;

}

// Computes the error for a given reference
void TaskSelfCollision::getError(Eigen::VectorXd & error_out, bool compute){
	if (compute){
		this->computeError();
	}
	error_out = error_;
}

// Sets the task error manually
void TaskSelfCollision::setError(const Eigen::VectorXd & error_in){
	std::cout << "Warning! Task " << task_name << " has no setError(error)" << " implementation" << std::endl;
}