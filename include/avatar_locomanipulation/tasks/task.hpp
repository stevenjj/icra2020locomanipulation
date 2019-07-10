#ifndef ALM_TASK_MAIN_H
#define ALM_TASK_MAIN_H

#include <Eigen/Dense>

#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <avatar_locomanipulation/helpers/orientation_utils.hpp>

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
	virtual void setReference(const Eigen::VectorXd & vec_ref_in);
	virtual void setReference(const Eigen::Quaterniond & quat_ref_in);	
	virtual void setReference(const Eigen::VectorXd & vec_ref_in, const Eigen::Quaterniond & quat_ref_in);

	// Gets the currently set References
	virtual void getReference(Eigen::VectorXd & vec_ref_out);
	virtual void getReference(Eigen::VectorXd & vec_ref_out, Eigen::Quaterniond & quat_ref_out);
	virtual void getReference(Eigen::Quaterniond & quat_ref_out);

	// Computes the error for a given reference
	virtual void computeError();
	// Gets the current error
	virtual void getError(Eigen::VectorXd & error_out, bool compute=true);
	// Sets the task error manually
	virtual void setError(const Eigen::VectorXd & error_in);


	int task_dim = 0;
	std::shared_ptr<ValkyrieModel> robot_model;
	std::string task_name = "empty task";
	std::string frame_name = "no frame";

protected:
	Eigen::VectorXd vec_ref_;
	Eigen::Quaterniond quat_ref_;
	Eigen::VectorXd error_;

};

#endif