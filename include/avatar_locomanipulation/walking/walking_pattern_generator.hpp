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

  std::vector<Eigen::Vector3d> rvp_list; // List of virtual repelant points.
  std::vector<Eigen::Vector3d> dcm_ini_list; // List of initial DCM states 
  std::vector<Eigen::Vector3d> dcm_eos_list; // List of end-of-step DCM states


  // DCM parameters:
  double gravity = 9.81;
  double z_vrp = 0.95; // desired VRP height / CoM height
  double b = std::sqrt(z_vrp/gravity); // time constant of DCM dynamics  

  double t_it = 0.9; // initial transfer time
  double t_ds = 0.9; // time in double support
  double t_ss = 1.2; // time in single support

  double t_swing = 1.2; // total swing time
  double transfer_time = 0.9;
  double alpha = 0.5; // ratio between double initial and final double support time

  double swing_height = 0.05; // swing height in meters.

  // Swing trajectory calculation
  void computeSE3_trajectory(const Footstep & init_stance_location, const Footstep & landing_location);

  // DCM trajectory calculation
  void computeDCM_ini_i(const Eigen::Vector3d & r_vrp_d_i, const double & t_step, const Eigen::Vector3d & zeta_eos_i);


  void computeDCM_states(const Eigen::Vector3d & initial_rvp, const Eigen::Vector3d & initial_foot_stance);

};

#endif