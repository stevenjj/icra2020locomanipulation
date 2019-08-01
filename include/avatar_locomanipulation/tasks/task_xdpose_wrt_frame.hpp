#ifndef ALM_XDPOSE_WRT_FRAME_TASK_H
#define ALM_XDPOSE_WRT_FRAME_TASK_H

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/data_types/footstep.hpp>

class TaskXDPosewrtFrame: public Task6DPose{
public:
	TaskXDPosewrtFrame(std::shared_ptr<RobotModel> & input_model, 
					   std::vector<int> input_task_dimensions,
					   const std::string & input_frame_name, 
					   const std::string & input_wrt_frame_name);
	virtual ~TaskXDPosewrtFrame();

	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	// Computes the error for a given reference
	virtual void computeError();

private:
	std::string wrt_frame_name;
	std::vector<int> task_dimensions;

	Eigen::MatrixXd J_tmp;
	Eigen::MatrixXd J_wrt;
	Eigen::MatrixXd J_out;

	Eigen::MatrixXd Jdot_tmp;
	Eigen::MatrixXd Jdot_wrt;
	Eigen::MatrixXd Jdot_out;

	Eigen::Vector3d frame_pos;
	Eigen::Quaterniond frame_quat;

	Eigen::Vector3d des_pos;
	Eigen::Quaterniond des_quat;

	Eigen::VectorXd error_tmp;
};

#endif