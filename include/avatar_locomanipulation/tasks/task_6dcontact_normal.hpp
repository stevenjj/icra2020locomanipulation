#ifndef ALM_6D_CONTACT_NORMAL_TASK_H
#define ALM_6D_CONTACT_NORMAL_TASK_H

#include <avatar_locomanipulation/tasks/task_6dpose.hpp>

class Task6DContactNormalTask: public Task6DPose{
public:
	Task6DContactNormalTask(std::shared_ptr<ValkyrieModel> & input_model, const std::string & input_frame_name, 
						 const Eigen::Vector3d & normal_vec, const Eigen::Vector3d normal_vec_tail);
	virtual ~Task6DContactNormalTask();

	// Computes the error for a given reference
	virtual void computeError();

private:
	Eigen::Vector3d normal_vec_hat_;
	Eigen::Vector3d normal_vec_tail_;

	Eigen::Vector3d des_pos;
	Eigen::Quaterniond des_quat;

    Eigen::Matrix3d R_frame_ori_;
    Eigen::Vector3d z_hat_;

	Eigen::Vector3d omega_hat_;
	double theta_angle_;
	Eigen::AngleAxisd omega_;
	Eigen::Quaterniond quat_omega_;


	double dist_to_plane;
};

#endif