#ifndef ALM_4DCONTACT_NORMAL_TASK_H
#define ALM_4DCONTACT_NORMAL_TASK_H

#include <avatar_locomanipulation/tasks/task.hpp>

class Task4DContactNormalTask: public Task{
public:
	Task4DContactNormalTask(std::shared_ptr<RobotModel> & input_model, const std::string & input_frame_name, 
						    const Eigen::Vector3d & normal_vec, const Eigen::Vector3d normal_vec_tail);	

	virtual ~Task4DContactNormalTask();

	virtual void getTaskJacobian(Eigen::MatrixXd & J_task);
	virtual void getTaskJacobianDot(Eigen::MatrixXd & Jdot_task);

	// Computes the error for a given reference
	virtual void computeError();
	// Gets the current error
	virtual void getError(Eigen::VectorXd & error_out, bool compute=true);

protected:
	Eigen::Vector3d cur_pos_;
	Eigen::Quaterniond quat_current_;
	Eigen::Vector3d quat_error_;

	Eigen::MatrixXd J_tmp;
	Eigen::MatrixXd Jdot_tmp;

	// Member variables for computing planar contact error
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

	Eigen::Vector3d closest_point_on_plane_;
	double dist_to_plane;



};

#endif