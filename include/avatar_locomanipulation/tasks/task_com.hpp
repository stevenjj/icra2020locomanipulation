#ifndef ALM_COM_TASK_H
#define ALM_COM_TASK_H

#include <avatar_locomanipulation/tasks/task.hpp>

class TaskCOM: public Task{
public:
	TaskCOM(std::shared_ptr<RobotModel> & input_model);

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

	// Sets a reference for the task
	virtual void setReference(const Eigen::VectorXd & vec_ref_in);

	// Gets the currently set References
	virtual void getReference(Eigen::VectorXd & vec_ref_out);

	// Computes the error for a given reference
	virtual void computeError();
	// Gets the current error
	virtual void getError(Eigen::VectorXd & error_out, bool compute=true);


protected:
	Eigen::Vector3d cur_pos_;

};

#endif