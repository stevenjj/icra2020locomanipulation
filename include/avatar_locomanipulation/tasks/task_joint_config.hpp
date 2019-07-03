#ifndef ALM_JOINT_CONFIG_TASK_H
#define ALM_JOINT_CONFIG_TASK_H

#include <avatar_locomanipulation/tasks/task.hpp>
#include <vector>

class TaskJointConfig: public Task{
public:
	TaskJointConfig(std::shared_ptr<ValkyrieModel> & input_model, const std::vector<std::string> & joint_names);

	virtual ~TaskJointConfig();

	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	Eigen::MatrixXd J_config;
	Eigen::MatrixXd Jdot_config;
};

#endif