#ifndef COLLISION_ENVIRONMENT_H
#define COLLISION_ENVIRONMENT_H

#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Robot Model
#include <avatar_locomanipulation/models/robot_model.hpp>



class CollisionEnvironment
{
private:
  std::shared_ptr<RobotModel> valkyrie, object;
public:

  CollisionEnvironment(std::shared_ptr<RobotModel> & val, std::shared_ptr<RobotModel> & obj);
  ~CollisionEnvironment();

  // builds vectors from object to joints of interest using maps defined by find_world_positions and find_near_points
  // Input: robot config, object config
  void build_directed_vectors(Eigen::VectorXd & q, Eigen::VectorXd & obj_config);

  // fills the map from joint name to world position
  std::map<std::string, Eigen::Vector3d> find_world_positions();

  // fills map from object link name to nearest point on that body to pelvis
  // Input: robot config, object config
  std::map<std::string, Eigen::Vector3d> find_near_points(Eigen::VectorXd & q, Eigen::VectorXd & obj_config);
	
  // computes collision and outputs any contacts
  void compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config);
};




#endif
