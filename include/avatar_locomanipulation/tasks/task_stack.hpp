#ifndef ALM_STACK_TASK_H
#define ALM_STACK_TASK_H

#include <avatar_locomanipulation/tasks/task.hpp>

class TaskStack: public Task{
public:
	TaskStack(std::shared_ptr<ValkyrieModel> & input_model, const std::vector< std::shared_ptr<Task> > & task_list_input);

	virtual ~TaskStack();

	// Warning: robot_model->updateFullKinematics(q) and 
	//			robot_model->computeCoMJacobian() must have been called first
	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);

	// Warning: robot_model->updateFullKinematics(q), 
	//			robot_model->computeCoMJacobian(),
	//	        robot_model->updateKinematicsDerivatives(q, qdot, qddot);
	//  		robot_model->computeCoMJacobianDot(q, qdot);
	//			must have been called first	
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	std::vector< std::shared_ptr<Task> > task_list;
	std::vector< Eigen::MatrixXd > vec_Jtmp;
	std::vector< Eigen::MatrixXd > vec_Jdottmp;


	Eigen::MatrixXd J_stack;
	Eigen::MatrixXd Jdot_stack;

};

#endif