#ifndef EUCLIDEAN_TRAJECTORY_H
#define EUCLIDEAN_TRAJECTORY_H

#include <Eigen/Dense>
#include <iostream>


class TrajEuclidean{
public:
	TrajEuclidean();
	TrajEuclidean(const int & dim_in, const int & N_size_in, const double & dt_in);
	virtual ~TrajEuclidean();

	// Getter functions
	double get_dt();
	int get_dim();
	int get_trajectory_length();

	virtual void get_pos(const int & index_in, Eigen::Ref<Eigen::VectorXd> pos_out);
	virtual void get_vel(const int & index_in, Eigen::Ref<Eigen::VectorXd> vel_out);
	virtual void get_acc(const int & index_in, Eigen::Ref<Eigen::VectorXd> acc_out);

	// All of the get_next functions increment the internal index counter
	virtual void get_next_pos(Eigen::Ref<Eigen::VectorXd> pos_out);
	virtual void get_next_vel(Eigen::Ref<Eigen::VectorXd> vel_out);
	virtual void get_next_acc(Eigen::Ref<Eigen::VectorXd> acc_out);

	virtual void get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out);
	virtual void get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out, Eigen::Ref<Eigen::VectorXd> vel_out);
	virtual void get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out, Eigen::Ref<Eigen::VectorXd> vel_out, Eigen::Ref<Eigen::VectorXd> acc_out);

	// Setter functions
	void set_dim_N_dt(const int & dim_in, const int & N_size_in, const double & dt_in);
	void set_pos(const int & index, const Eigen::VectorXd & pos_in);
	void set_vel(const int & index, const Eigen::VectorXd & vel_in);
	void set_acc(const int & index, const Eigen::VectorXd & acc_in);

	// resets the internal index counter
	void reset_index();

protected:
	std::vector<Eigen::VectorXd> pos;
	std::vector<Eigen::VectorXd> vel;
	std::vector<Eigen::VectorXd> acc;

	int N_size;
	int dim;
	double dt;
	int index = 0;

	void increment_index();
};


class TrajEuclideanConstant: public TrajEuclidean{
public:
	TrajEuclideanConstant();
	TrajEuclideanConstant(const Eigen::VectorXd vec_in);
	~TrajEuclideanConstant();

	virtual void get_pos(const int & index_in, Eigen::Ref<Eigen::VectorXd> pos_out);
	virtual void get_vel(const int & index_in, Eigen::Ref<Eigen::VectorXd> vel_out);
	virtual void get_acc(const int & index_in, Eigen::Ref<Eigen::VectorXd> acc_out);

	virtual void get_next_pos(Eigen::Ref<Eigen::VectorXd> pos_out);
	virtual void get_next_vel(Eigen::Ref<Eigen::VectorXd> vel_out);
	virtual void get_next_acc(Eigen::Ref<Eigen::VectorXd> acc_out);

	virtual void get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out);
	virtual void get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out, Eigen::Ref<Eigen::VectorXd> vel_out);
	virtual void get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out, Eigen::Ref<Eigen::VectorXd> vel_out, Eigen::Ref<Eigen::VectorXd> acc_out);

private:
	Eigen::VectorXd vec_constant;
	Eigen::VectorXd vec_zero;	

};

#endif
