#ifndef ALM_6DPOSE_TASK_H
#define ALM_6DPOSE_TASK_H

#include <avatar_locomanipulation/tasks/task.hpp>

class Task6DPose: public Task{
public:
	Task6DPose(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name);

	virtual ~Task6DPose();

	// Warning: robot_model->updateFullKinematics(q) and 
	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);

	// Warning: robot_model->updateFullKinematics(q), 
	//	        robot_model->updateKinematicsDerivatives(q, qdot, qddot);
	//			must have been called first	
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);


	// Sets a reference for the task
	virtual void setReference(const Eigen::VectorXd & vec_ref_in);
	virtual void setReference(const Eigen::Quaterniond & quat_ref_in);	
	virtual void setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in);

	// Gets the currently set References
	virtual void getRef(Eigen::VectorXd & vec_ref_out);
	virtual void getRef(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out);
	virtual void getRef(Eigen::Quaterniond & quat_ref_out);

	// Computes the error for a given reference
	virtual void computeError();
	// Gets the current error
	virtual void getError(Eigen::VectorXd & error_out, bool compute=true);
	// Sets the task error manually
	virtual void setError(const Eigen::VectorXd & error_in);

protected:
	Eigen::Vector3d cur_pos_;
	Eigen::Quaterniond quat_current_;
	Eigen::Vector3d quat_error_;


};

#endif