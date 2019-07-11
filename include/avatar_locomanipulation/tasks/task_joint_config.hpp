#ifndef ALM_JOINT_CONFIG_TASK_H
#define ALM_JOINT_CONFIG_TASK_H

#include <avatar_locomanipulation/tasks/task.hpp>
#include <vector>
#include <string>

class TaskJointConfig: public Task{
public:
	TaskJointConfig(std::shared_ptr<ValkyrieModel> & input_model, const std::vector<std::string> & joint_names);

	virtual ~TaskJointConfig();

	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	virtual void setReference(const Eigen::VectorXd & vec_ref_in);
	virtual void getReference(Eigen::VectorXd & vec_ref_out);
	virtual void computeError();
	virtual void getError(Eigen::VectorXd & error_out, bool compute=true);


	Eigen::MatrixXd J_config;
	Eigen::MatrixXd Jdot_config;
private:
	Eigen::VectorXd cur_joint_pos;
	std::vector<std::string> joint_names_;
};

#endif