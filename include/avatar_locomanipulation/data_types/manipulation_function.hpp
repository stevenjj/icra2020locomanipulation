#ifndef MANIPULATION_FUNCTION_OBJECT_H
#define MANIPULATION_FUNCTION_OBJECT_H

#include <Eigen/Dense>
#include <iostream>
#include <string>

#include <avatar_locomanipulation/cubic_interpolation_module/cubic_interpolation_six_dim_vec.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#define MANIPULATE_TYPE_RIGHT_HAND 0
#define MANIPULATE_TYPE_LEFT_HAND 1
#define MANIPULATE_TYPE_BOTH_HANDS 2

// Data container for a manipulation function object
class ManipulationFunction{
public:
  ManipulationFunction();
  ManipulationFunction(std::string filename);
  ~ManipulationFunction(); 

  void setWorldTransform(const Eigen::Vector3d & translation, const Eigen::Quaterniond & orientation);
  void getPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out);

  void setWaypointsFromYaml(std::string filename);

  void setRightWaypointsFromYaml(std::string filename);
  void setLeftWaypointsFromYaml(std::string filename);

  void getRightHandPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out);
  void getLeftHandPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out);
 
  int getManipulationType();

private:
  void common_initialization();

  int manipulation_type;

  std::shared_ptr<CubicInterpolationSixDimVec> f_rh_s;
  std::shared_ptr<CubicInterpolationSixDimVec> f_lh_s;

  std::string rh_waypoint_list_yaml_filename;
  std::string lh_waypoint_list_yaml_filename;
  int N_waypoints;

  // Local frame position and orientation of the interpolated waypoint
  Eigen::Vector3d rhand_local_pos;
  Eigen::Quaterniond rhand_local_ori;

  Eigen::Vector3d lhand_local_pos;
  Eigen::Quaterniond lhand_local_ori;


  bool right_hand_waypoints_set = false;
  bool left_hand_waypoints_set = false;

  // Homogenous transform w.r.t world frame to convert
  Eigen::Vector3d transform_translation;
  Eigen::Quaterniond transform_orientation;


};

#endif