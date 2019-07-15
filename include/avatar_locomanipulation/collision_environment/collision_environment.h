#ifndef COLLISION_ENVIRONMENT_H
#define COLLISION_ENVIRONMENT_H

#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Robot Model
#include <avatar_locomanipulation/models/robot_model.hpp>
// Directed Vectors
#include <avatar_locomanipulation/collision_environment/directed_vectors.hpp>



class CollisionEnvironment
{
private:
  // for internally filling the vector of directed vectors  
  // Members: std::string to - name of joint vector points to
  //          std::string from - name of joint vector comes from
  //          Eigen::Vector3d direction - normalized direction of vector
  //          double magnitude - magnitude of the vector for use with potential
  DirectedVectors dvector;


  // builds a map of names for easy use to frame name for getting current world position
  std::map<std::string, std::string> make_map_to_frame_names();
  std::map<std::string, std::string> make_map_to_frame_names_subset();


  // builds a map of names for easy use to collision body name for getting nearest point
  // i.e. "rfoot" to "rightFoot_0"
  std::map<std::string, std::string> make_map_to_collision_body_names();


  // appends the models internally
  // Input: robot config, object config 
  // Used in distance computation, which does not require the appended config
  std::shared_ptr<RobotModel> append_models(Eigen::VectorXd & q, Eigen::VectorXd & obj_config);

public:

  // The two robot models which we are adding to the environment
  std::shared_ptr<RobotModel> valkyrie, object;

  // The vector of directed vectors and related information
  std::vector<DirectedVectors> directed_vectors, self_directed_vectors;
  

  // Constructor fills the local data for valkyrie, object RobotModels
  // Inputs: RobotModel valkyrie
  //         RobotModel environmental_object
  CollisionEnvironment(std::shared_ptr<RobotModel> & val, std::shared_ptr<RobotModel> & obj);

  // Constructor for only valkyrie model
  CollisionEnvironment(std::shared_ptr<RobotModel> & val);


  ~CollisionEnvironment();


  // builds vectors from object to joints of interest using maps defined by find_world_positions and find_near_points
  // Input: robot config, object config
  void build_directed_vectors(Eigen::VectorXd & q, Eigen::VectorXd & obj_config);


  // builds vectors from joints to joints 
  // Input: robot config, object config
  void build_self_directed_vectors(Eigen::VectorXd & q);


  // fills the map from joint name to world position
  std::map<std::string, Eigen::Vector3d> find_world_positions();


  // fills the map from joint name to world position for our subset used in self collision checking
  std::map<std::string, Eigen::Vector3d> find_world_positions_subset();


  // fills map from object link name to nearest point on that body to pelvis
  // Input: appended model,
  //        object link name on which we are interested in near point
  std::map<std::string, Eigen::Vector3d> find_near_points(std::shared_ptr<RobotModel> & appended, std::string name);
	

  // computes collision and outputs any contacts
  void compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config);


  // gives us a command for dx to move away from self collision
  std::vector<Eigen::Vector3d> self_collision_dx();

  // These functions will be called by build_self_directed_vectors
  // Or they may be called explicitly by tasks computeError
  // Input: subset of world positions
  void build_directed_vector_to_rhand(std::map<std::string, Eigen::Vector3d> world_positions);
  void build_directed_vector_to_lhand(std::map<std::string, Eigen::Vector3d> world_positions);
  void build_directed_vector_to_elbows(std::map<std::string, Eigen::Vector3d> world_positions);
  void build_directed_vector_to_lknee(std::map<std::string, Eigen::Vector3d> world_positions);
  void build_directed_vector_to_rknee(std::map<std::string, Eigen::Vector3d> world_positions);
  void build_directed_vector_to_head(std::map<std::string, Eigen::Vector3d> world_positions);
};




#endif
