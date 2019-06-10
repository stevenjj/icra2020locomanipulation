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

  Eigen::Vector3d position;
  Eigen::Quaternion<double>  orientation;

};

#endif