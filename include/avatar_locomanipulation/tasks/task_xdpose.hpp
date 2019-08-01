#ifndef ALM_XDPOSE_TASK_H
#define ALM_XDPOSE_TASK_H

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/data_types/footstep.hpp>

class TaskXDPose: public Task6DPose{
public:
	TaskXDPose(std::shared_ptr<RobotModel> & input_model, 
					   std::vector<int> input_task_dimensions,
					   const std::string & input_frame_name);
	virtual ~TaskXDPose();

	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	// Computes the error for a given reference
	virtual void computeError();

private:
	std::vector<int> task_dimensions;

	Eigen::VectorXd error_tmp;

	Eigen::MatrixXd J_tmp;
	Eigen::MatrixXd Jdot_tmp;

	Eigen::MatrixXd J_out;
	Eigen::MatrixXd Jdot_out;

};

#endif