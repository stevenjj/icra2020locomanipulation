#ifndef WALKING_PATTERN_GENERATOR_H
#define WALKING_PATTERN_GENERATOR_H

#include <Eigen/Dense>
#include <string>
#include <avatar_locomanipulation/data_types/footstep.hpp>
#include <iostream>

class WalkingPatternGenerator{
public:
  WalkingPatternGenerator();
  ~WalkingPatternGenerator(); 

  std::vector<Footstep> footstep_list;

  double swing_time;
  double transfer_time;

  void computeSE3_trajectory(const Footstep & init_stance_location, const Footstep & landing_location);

};

#endif