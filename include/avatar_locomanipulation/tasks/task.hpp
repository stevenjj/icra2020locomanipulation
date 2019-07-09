#ifndef ALM_TASK_MAIN_H
#define ALM_TASK_MAIN_H

#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <Eigen/Dense>
#include <string>
#include <iostream>

class Task{
public:
	Task();
	Task(std::shared_ptr<ValkyrieModel> & input_model);
	virtual ~Task();

	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	// Sets a reference for the task
	// vec_ref can be a position or a custom
	virtual void setReference(const Eigen::VectorXd & vec_ref);
	virtual void setReference(const Eigen::Quaterniond & quat_ref);	
	virtual void setReference(const Eigen::VectorXd & vec_ref, const Eigen::Quaterniond & quat_ref);

	// Gets the currently set References
	virtual void getRef(Eigen::VectorXd & vec_ref_out);
	virtual void getRef(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out);
	virtual void getRef(Eigen::Quaterniond & quat_ref_out);

	// Computes the error for a given reference
	virtual void getError(Eigen::VectorXd & error_out);
	// Sets the task error manually
	virtual void setError(const Eigen::VectorXd & error);


	int task_dim = 0;
	std::shared_ptr<ValkyrieModel> robot_model;
	std::string task_name = "empty task";
	std::string frame_name = "no frame";



};

#endif