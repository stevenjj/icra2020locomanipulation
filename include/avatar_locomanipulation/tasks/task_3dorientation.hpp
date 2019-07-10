#ifndef ALM_3DORIENTATION_TASK_H
#define ALM_3DORIENTATION_TASK_H

#include <avatar_locomanipulation/tasks/task.hpp>

class Task3DOrientation: public Task{
public:
	Task3DOrientation(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name);

	virtual ~Task3DOrientation();

	// Warning: robot_model->updateFullKinematics(q) and 
	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);

	// Warning: robot_model->updateFullKinematics(q), 
	//	        robot_model->updateKinematicsDerivatives(q, qdot, qddot);
	//			must have been called first	
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	// Set the reference
	virtual void setReference(const Eigen::Quaterniond & quat_ref_in);	
	
	// Get the reference value
	virtual void getReference(Eigen::Quaterniond & quat_ref_out);
	virtual void computeError();
	virtual void getError(Eigen::VectorXd & error_out, bool compute=true);


private:
	Eigen::MatrixXd J_tmp;
	Eigen::MatrixXd Jdot_tmp;


};

#endif