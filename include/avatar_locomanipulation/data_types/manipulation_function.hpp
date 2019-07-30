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

  void getPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out);
  void setWaypointsFromYaml(std::string filename);
 
private:
  std::shared_ptr<CubicInterpolationSixDimVec> f_s;
  std::string waypoint_list_yaml_filename;
  int N_waypoints;


};

#endif