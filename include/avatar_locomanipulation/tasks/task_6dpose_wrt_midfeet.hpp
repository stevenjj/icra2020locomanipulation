#ifndef ALM_6DPOSE_WRT_MIDFEET_TASK_H
#define ALM_6DPOSE_WRT_MIDFEET_TASK_H

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/data_types/footstep.hpp>

class Task6DPosewrtMidFeet: public Task6DPose{
public:
	Task6DPosewrtMidFeet(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name);
	virtual ~Task6DPosewrtMidFeet();

	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	// Computes the error for a given reference
	virtual void computeError();

private:
	std::string left_foot_frame;
	std::string right_foot_frame;

	Eigen::MatrixXd J_tmp;
	Eigen::MatrixXd J_lf;
	Eigen::MatrixXd J_rf;

	Eigen::MatrixXd Jdot_tmp;
	Eigen::MatrixXd Jdot_lf;
	Eigen::MatrixXd Jdot_rf;

	Footstep left_foot;
	Footstep right_foot;
	Footstep midfeet;

	Eigen::Vector3d des_pos;
	Eigen::Quaterniond des_quat;
};

#endif