#ifndef ALM_TASK_MAIN_H
#define ALM_TASK_MAIN_H

#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <Eigen/Dense>
#include <string>
#include <iostream>

class TaskMain{
public:
	TaskMain();
	TaskMain(std::shared_ptr<ValkyrieModel> & input_model);
	virtual ~TaskMain();

	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	int task_dim = 0;
	std::shared_ptr<ValkyrieModel> robot_model;
	std::string task_name = "empty task";
	std::string frame_name = "no frame";

};

#endif