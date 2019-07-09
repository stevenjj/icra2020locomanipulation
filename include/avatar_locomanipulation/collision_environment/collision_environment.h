#ifndef COLLISION_ENVIRONMENT_H
#define COLLISION_ENVIRONMENT_H

#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Robot Model
#include <avatar_locomanipulation/models/robot_model.hpp>



class Collision
{
private:
	

public:
	Collision();
	~Collision();
	void build_valkyrie_model_and_geom();
	void build_box_planar_joint_model_and_geom();
	void build_cart_model_and_geom();
	void append_models(std::shared_ptr<Collision> & parent, std::shared_ptr<Collision> & child, std::shared_ptr<Collision> & appended);// Parent, child
	void set_configuration_vector(Eigen::VectorXd & config_vec);// vector of configurations
	void compute_distances();
	void compute_near_point(std::string name1, std::string name2, Eigen::Vector3d & near_point, Eigen::VectorXd q_start);
	void compute_collision(); 
	pinocchio::fcl::Quaternion3f makeQuat(double w, double x, double y, double z);
	int get_nq();
	void get_position_of_joints(Eigen::VectorXd q_start, std::map<std::string, Eigen::Vector3d> & positions);
};




#endif
