#ifndef ALM_COM_TASK_H
#define ALM_COM_TASK_H

#include <avatar_locomanipulation/tasks/task.hpp>

class TaskCOM: public Task{
public:
	TaskCOM(std::shared_ptr<ValkyrieModel> & input_model);

	virtual ~TaskCOM();

	// Warning: robot_model->updateFullKinematics(q) and 
	//			robot_model->computeCoMJacobian() must have been called first
	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);

	// Warning: robot_model->updateFullKinematics(q), 
	//			robot_model->computeCoMJacobian(),
	//	        robot_model->updateKinematicsDerivatives(q, qdot, qddot);
	//  		robot_model->computeCoMJacobianDot(q, qdot);
	//			must have been called first	
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

};

#endif