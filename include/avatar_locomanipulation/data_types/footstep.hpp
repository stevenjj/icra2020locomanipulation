#ifndef FOOTSTEP_OBJECT_H
#define FOOTSTEP_OBJECT_H

#include <Eigen/Dense>
#include <iostream>
#include <string>

// Data container for a footstep object

class Footstep{
public:
  Footstep();
  ~Footstep(); 

  // Position and orientation of the sole of the footstep
  Eigen::Vector3d position;
  Eigen::Quaternion<double>  orientation;

  // length and width of the soles
  double sole_length;
  double sole_width

};

#endif