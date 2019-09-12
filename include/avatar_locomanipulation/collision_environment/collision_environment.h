#ifndef COLLISION_ENVIRONMENT_H
#define COLLISION_ENVIRONMENT_H

#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Robot Model
#include <avatar_locomanipulation/models/robot_model.hpp>
// Directed Vectors
#include <avatar_locomanipulation/collision_environment/directed_vectors.hpp>
#include <math.h>



class CollisionEnvironment
{
private:
  // for internally filling the vector of directed vectors  
  // Members: std::string to - name of joint/link vector points to
  //          std::string from - name of joint vector comes from
  //          Eigen::Vector3d direction - normalized direction of vector
  //          double magnitude - magnitude of the vector for use with potential
  DirectedVectors dvector;

  // Vector containing the names of all object collision links
  std::vector<std::string> object_links;

  // a map from collision body names to frame names
  std::map<std::string, std::string> collision_to_frame;

  // The distance for a link to move away from the body if inside the safety distance
  double max_scaling_distance = 0.15;

  // If we input an object RobotModel into the environment, then this is true and 
  //  map_collision_names_to_frame_names also includes the object frame names
  bool object_flag = false;

  // String holding the prefix given to frame names
  // Used in creating map from collision to frame names
  std::string prefix_;


  // appends the models internally
  // first_time appends valkyrie to appended
  // subsequently appends object onto end of appended
  void append_models();



  // Input: empty map string to string
  // fills the map with key being the collision body names and value being the corresponding frame name
  void map_collision_names_to_frame_names();

  // Makes a list of frame names of interest for build_point_list_directed_vectors
  std::vector<std::string> make_point_collision_list();

  
  // Create map for self collision to generakize self directed vectors
  void generalize_build_self_directed_vectors();

  // Create map for object collision to generalize the object directed vectors
  void generalize_build_object_directed_vectors();


  // Builds a directed vector using world positions for the case when two bodies are in collision
  // called internally from build_directed_vectors_name
  // Input: name of two collision bodies
  void get_dvector_collision_links(const std::string & from_name, const std::string & to_name);

  
  void get_object_links();

public:
  // Potential function scaling distance
  double eta;

  // Variable used to keep track of the length of the combined q_currents for every added object
  double object_q_counter = 0;

  // The tolerance distance for avoiding collisions
  double safety_dist_normal = 0.075;
  double safety_dist_collision = 0.2;

  // boolean such that an empty appended robotmodel becomes appended with valkyrie on collision environment init
  bool first_time = false;

  // index of the closest directedvector for calculating potential
  int closest;

  // Generalizes our build_self_directed_vectors
  // link_to_collision_names["rightPalm_0"] = {"leftPalm_0", ....}
  std::map<std::string, std::vector<std::string> > link_to_collision_names;

  std::map<std::string, std::vector<std::string> > link_to_object_collision_names;


  // The two robot models which we are adding to the environment
  std::shared_ptr<RobotModel> valkyrie, object, appended;

  // The vector of directed vectors and related information
  std::vector<DirectedVectors>  directed_vectors;

  // Constructor initializes appended model to be val
  // Input: - valkyrie RobotModel
  CollisionEnvironment(std::shared_ptr<RobotModel> & val);


  ~CollisionEnvironment();


  // Called by build_____directed_vectors, provides that function with pairs of nearest_points
  // Inputs: - List of collision object names, with the first being used as the "to" object
  //         - (Empty) map from names of "from" collision links to the nearest point on those objects
  //         - (Empty) map from names of "from"collision links to nearest points on the "to" objects
  void find_near_points(std::string & interest_link, const std::vector<std::string>  & list, std::map<std::string, Eigen::Vector3d> & from_near_points, std::map<std::string, Eigen::Vector3d> & to_near_points);


  // Given a frame name from a task it will build the directed vectors to that link
  // Input: - Relevant frame name from the task_selfcollision
  //        - q_update, the current robot config
  void build_self_directed_vectors(const std::string & frame_name, Eigen::VectorXd & q_update);

  // Given a list of points in space, this builds directed vectors from these points to set of links
  //  to avoid those points
  // Inputs: - list of 3D points from which to build dvectors
  //         - q_update, the current robot config 
  void build_point_list_directed_vectors(const std::vector<Eigen::Vector3d> & point_list_in, Eigen::VectorXd & q_update, std::string & frame);

 
  // Used by self collision and object collision tasks to calculate the potential field
  double get_collision_potential();
  

  // Sets the safety distance between links when not in collision
  void set_safety_distance_normal(double safety_dist_normal_in);

  // Sets the safety distance between links when in collision
  void set_safety_distance_collision(double safety_dist_collision_in);

  // Given a frame name from a task it will build directed vectors to that link from all object links
  // Input: - Relevant frame name from the task_objectcollision
  void build_object_directed_vectors(std::string & frame_name, Eigen::VectorXd & q_update);

  // Given an object RobotModel and its q_start, appends this and adds it to appended
  void add_new_object(std::shared_ptr<RobotModel> & obj, const Eigen::VectorXd & q_start, std::string & prefix);


  // // computes collision and outputs any contacts
  // void compute_collision(Eigen::VectorXd & q, Eigen::VectorXd & obj_config);

};




#endif
