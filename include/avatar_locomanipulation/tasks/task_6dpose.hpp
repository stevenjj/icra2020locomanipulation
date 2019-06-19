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

};

#endif