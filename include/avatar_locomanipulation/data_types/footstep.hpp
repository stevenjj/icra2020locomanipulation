#ifndef FOOTSTEP_OBJECT_H
#define FOOTSTEP_OBJECT_H

#include <Eigen/Dense>
#include <iostream>
#include <string>

#define LEFT_FOOTSTEP 0
#define RIGHT_FOOTSTEP 1
// Data container for a footstep object


class Footstep{
public:
  Footstep();
  Footstep(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in);

  ~Footstep(); 

  // Position and orientation of the sole of the footstep
  Eigen::Vector3d position;
  Eigen::Quaternion<double>  orientation;

  Eigen::Matrix3d R_ori;

  // Left or right side
  int robot_side;

  int getSide();
  void printInfo();

  // length and width of the soles
  double sole_length;
  double sole_width;

private:
  void common_initialization();

};

#endif