#ifndef COLLISION_CLASS_H
#define COLLISION_CLASS_H

#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Multibody
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
// Parsers
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
// Algorithms
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
// Spatial
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
// Standard
#include <math.h> 
#include <iostream>
#include <iomanip>
#include <vector>
#include <boost/shared_ptr.hpp>

class Collision
{
private:
	pinocchio::Model model;
	pinocchio::GeometryModel geomModel;
	pinocchio::fcl::CollisionResult result;
	pinocchio::fcl::DistanceResult dresult;
	std::vector<pinocchio::fcl::Contact> contacts;
	std::string srdf_filename; // used in cases with robot
	std::string filename; // used in cases with robot
	Eigen::VectorXd q;
	pinocchio::Model::JointIndex idx;

public:
	Collision();
	~Collision();
	void build_valkyrie_model_and_geom();
	void build_box_planar_joint_model_and_geom();
	void append_models(std::shared_ptr<Collision> & parent, std::shared_ptr<Collision> & child, std::shared_ptr<Collision> & appended);// Parent, child
	void set_configuration_vector(Eigen::VectorXd & config_vec);// vector of configurations
	void compute_distance();
	bool compute_collision(); // returns true if collision occurs
	pinocchio::fcl::Quaternion3f makeQuat(double w, double x, double y, double z);
	int get_nq();
};




#endif