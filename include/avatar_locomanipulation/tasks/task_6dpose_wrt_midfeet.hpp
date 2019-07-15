#ifndef ALM_6DPOSE_WRT_MIDFEET_TASK_H
#define ALM_6DPOSE_WRT_MIDFEET_TASK_H

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/data_types/footstep.hpp>

class Task6DPosewrtMidFeet: public Task6DPose{
public:
	Task6DPosewrtMidFeet(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name);
	virtual ~Task6DPosewrtMidFeet();

	// Computes the error for a given reference
	virtual void computeError();

private:
	std::string left_foot_frame;
	std::string right_foot_frame;

	Footstep left_foot;
	Footstep right_foot;
	Footstep midfeet;

	Eigen::Vector3d des_pos;
	Eigen::Quaterniond des_quat;
};

#endif