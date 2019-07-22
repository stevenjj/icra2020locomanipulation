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
  // Members: std::string to - name of joint/link vector points to
  //          std::string from - name of joint vector comes from
  //          Eigen::Vector3d direction - normalized direction of vector
  //          double magnitude - magnitude of the vector for use with potential
  DirectedVectors dvector;

  // a map from collision body names to frame names
  std::map<std::string, std::string> collision_to_frame;

  // The tolerance distance for avoiding self collisions
  double safety_dist = 0.05;

  // The distance for a link to move away from the body if inside the safety distance
  double max_scaling_distance = 0.15;

  // If we input an object RobotModel into the environment, then this is true and 
  //  map_collision_names_to_frame_names also includes the object frame names
  bool object_flag = false;


  // appends the models internally
  // Used in distance/collision computation, which does not require the appended config
  std::shared_ptr<RobotModel> append_models();



  // Input: empty map string to string
  // fills the map with key being the collision body names and value being the corresponding frame name
  void map_collision_names_to_frame_names();


  // Builds a directed vector using world positions for the case when two bodies are in collision
  // called internally from build_directed_vectors_name
  // Input: name of two collision bodies
  void get_dvector_collision_links(const std::string & from_name, const std::string & to_name);

  // same as the function get_dvector_collision_links, except it is used for instances of collision
  //  between the robot and the environmental object, hence we must use the appended RobotModel
  void get_dvector_collision_links_appended(std::shared_ptr<RobotModel> & appended, const std::string & from_name, const std::string & to_name);


  std::vector<std::string> get_object_links();

public:

  // The two robot models which we are adding to the environment
  std::shared_ptr<RobotModel> valkyrie, object;

  // The vector of directed vectors and related information
  std::vector<DirectedVectors>  directed_vectors;
  

  // Constructor fills the local data for valkyrie, object RobotModels
  // Inputs: RobotModel valkyrie
  //         RobotModel environmental_object
  CollisionEnvironment(std::shared_ptr<RobotModel> & val, std::shared_ptr<RobotModel> & obj);

  // Constructor for only valkyrie model
  CollisionEnvironment(std::shared_ptr<RobotModel> & val);


  ~CollisionEnvironment();


  // Inputs: - List of collision object names, with the first being used as the "to" object
  //         - (Empty) map from names of "from" collision links to the nearest point on those objects
  //         - (Empty) map from names of "from"collision links to nearest points on the "to" objects
  void find_self_near_points(std::vector<std::string> & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points);


  // Inputs: - List of collision object names, with the first being used as the "to" object
  //         - (Empty) map from names of "from" collision links to the nearest point on those objects
  //         - (Empty) map from names of "from"collision links to nearest points on the "to" objects
  void find_object_near_points(std::shared_ptr<RobotModel> & appended, std::vector<std::string> & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points);

  
  // Fills struct DirectedVector self_directed_vectors with the relevant vectors
  //  to the link indicated in the fnc name
  //    these directed vectors used in self_collision_dx to get the world dx
  void build_directed_vector_to_rhand();
  void build_directed_vector_to_lhand();
  void build_directed_vector_to_head();
  void build_directed_vector_to_lknee();
  void build_directed_vector_to_rknee();
  void build_directed_vector_to_rwrist();
  void build_directed_vector_to_lwrist();
  void build_directed_vector_to_relbow();
  void build_directed_vector_to_lelbow();


  // Fills the struct DirectedVector directed_vectors with the relevan vectors from each of the object
  //  links
  void build_object_directed_vectors(std::string & frame_name);


  // // computes collision and outputs any contacts
  // void compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config);


  // gives us a command for dx to move away from self collision
  std::vector<Eigen::Vector3d> get_collision_dx();
  

  // Sets the safety distance between robot links
  void set_safety_distance(double safety_dist_in);

  // Sets the max scaling factor for dx
  void set_max_scaling_distance(double & max_scaling_dist_in);
};




#endif
