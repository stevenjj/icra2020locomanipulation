#ifndef ORIENTATION_TRAJECTORY_H
#define ORIENTATION_TRAJECTORY_H

#include <Eigen/Dense>
#include <iostream>


class TrajOrientation{
public:
	TrajOrientation();
	TrajOrientation(const int & N_size_in, const double & dt_in);
	virtual ~TrajOrientation();

	// Getter functions
	double get_dt();
	int get_trajectory_length();

	virtual void get_quat(const int & index_in, Eigen::Quaterniond & quat_out);
	virtual void get_ang_vel(const int & index_in, Eigen::Vector3d & ang_vel_out);
	virtual void get_ang_acc(const int & index_in, Eigen::Vector3d & ang_acc_out);

	// All of the get_next functions increment the internal index counter
	virtual void get_next_quat(Eigen::Quaterniond & quat_out);
	virtual void get_next_ang_vel(Eigen::Vector3d & ang_vel_out);
	virtual void get_next_ang_acc(Eigen::Vector3d & ang_acc_out);

	virtual void get_next_data(Eigen::Quaterniond & quat_out);
	virtual void get_next_data(Eigen::Quaterniond & quat_out, Eigen::Vector3d & ang_vel_out);
	virtual void get_next_data(Eigen::Quaterniond & quat_out, Eigen::Vector3d & ang_vel_out, Eigen::Vector3d & ang_acc_out);

	// Setter functions
	void set_N_dt(const int & N_size_in, const double & dt_in);
	void set_quat(const int & index, const Eigen::Quaterniond & quat_in);
	void set_ang_vel(const int & index, const Eigen::Vector3d & ang_vel_in);
	void set_ang_acc(const int & index, const Eigen::Vector3d & ang_acc_in);

	// resets the internal index counter
	void reset_index();

protected:
	std::vector<Eigen::Quaterniond> quat;
	std::vector<Eigen::Vector3d> ang_vel;
	std::vector<Eigen::Vector3d> ang_acc;

	int N_size;
	double dt;
	int index = 0;

	void increment_index();
};


#endif
