#ifndef MANIPULATION_FUNCTION_OBJECT_H
#define MANIPULATION_FUNCTION_OBJECT_H

#include <Eigen/Dense>
#include <iostream>
#include <string>

#include <avatar_locomanipulation/cubic_interpolation_module/cubic_interpolation_six_dim_vec.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

// Data container for a manipulation function object
class ManipulationFunction{
public:
  ManipulationFunction();
  ManipulationFunction(std::string filename);
  ~ManipulationFunction(); 

  void setWorldTransform(const Eigen::Vector3d & translation, const Eigen::Quaterniond & orientation);
  void getPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out);
  void setWaypointsFromYaml(std::string filename);
 
private:
  void common_initialization();

  std::shared_ptr<CubicInterpolationSixDimVec> f_s;
  std::string waypoint_list_yaml_filename;
  int N_waypoints;

  // Local frame position and orientation of the interpolated waypoint
  Eigen::Vector3d local_pos;
  Eigen::Quaterniond local_ori;


  // Homogenous transform w.r.t world frame to convert
  Eigen::Vector3d transform_translation;
  Eigen::Quaterniond transform_orientation;


};

#endif