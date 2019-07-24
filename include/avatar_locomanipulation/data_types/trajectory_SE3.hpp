#ifndef SE3_TRAJECTORY_H
#define SE3_TRAJECTORY_H

#include <avatar_locomanipulation/data_types/trajectory_euclidean.hpp>
#include <avatar_locomanipulation/data_types/trajectory_orientation.hpp>

class TrajSE3{
public:
	TrajSE3();
	TrajSE3(const int & N_size_in, const double & dt_in);
	virtual ~TrajSE3();

	// Getter functions
	double get_dt();
	int get_trajectory_length();

	// All of the get_next functions increment the internal index counter
	virtual void get_pos(const int & index_in, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out);
	virtual void get_vel(const int & index_in, Eigen::Vector3d & vel_out, Eigen::Vector3d & ang_vel_out);
	virtual void get_acc(const int & index_in, Eigen::Vector3d & acc_out, Eigen::Vector3d & ang_acc_out);

	virtual void get_next_pos(Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out);
	virtual void get_next_vel(Eigen::Vector3d & vel_out, Eigen::Vector3d & ang_vel_out);
	virtual void get_next_acc(Eigen::Vector3d & acc_out, Eigen::Vector3d & ang_acc_out);

	virtual void get_next_data(Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out);

	virtual void get_next_data(Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out,
							   Eigen::Vector3d & vel_out, Eigen::Vector3d & ang_vel_out);

	virtual void get_next_data(Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out,
							   Eigen::Vector3d & vel_out, Eigen::Vector3d & ang_vel_out,
							   Eigen::Vector3d & acc_out, Eigen::Vector3d & ang_acc_out);

	// Setter functions
	void set_N_dt(const int & N_size_in, const double & dt_in);
	void set_dt(const double & dt_in);
	void set_pos(const int & index, const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in);
	void set_vel(const int & index, const Eigen::Vector3d & vel_in, const Eigen::Vector3d & ang_vel_in);
	void set_acc(const int & index, const Eigen::Vector3d & acc_in, const Eigen::Vector3d & ang_acc_in);

	// resets the internal index counter
	void reset_index();

	TrajEuclidean traj_pos;
	TrajOrientation traj_ori;

protected:
	int N_size;
	double dt;
};

#endif