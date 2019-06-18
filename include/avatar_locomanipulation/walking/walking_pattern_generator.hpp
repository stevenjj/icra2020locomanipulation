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

  std::vector<Eigen::Vector3d> rvrp_list; // List of virtual repelant points.
  std::vector<Eigen::Vector3d> dcm_ini_list; // List of initial DCM states 
  std::vector<Eigen::Vector3d> dcm_eos_list; // List of end-of-step DCM states

  Eigen::Vector3d start_stance_rvrp;
  Eigen::Vector3d start_stance_dcm_ini;  
  Eigen::Vector3d start_stance_dcm_eos;  

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

  // DCM trajectory calculation
  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        initial_footstance  - a footstep object describing the stance leg. 
  // populates this object's footstep_list, rvrp_list, dcm_ini_list, dcm_eos_list
  void initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, const Footstep & initial_footstance, bool clear_list=false);

  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        initial_footstance  - a footstep object describing the stance leg. 
  //        initial_rvrp        - an initial virtual repelant point (eg: average of the stance feet's rvrp). 
  // populates this object's footstep_list, rvrp_list, dcm_ini_list, dcm_eos_list
  void initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, const Footstep & initial_footstance, const Eigen::Vector3d & initial_rvrp);
 
  // Outputs the average r_vrp location given two footstances
  void get_average_rvrp(const Footstep & footstance_1, const Footstep & footstance_2, Eigen::Vector3d & average_rvrp);



  // Swing trajectory calculation
  void computeSE3_trajectory(const Footstep & init_stance_location, const Footstep & landing_location);

private:
  // input: r_vrp_d_i - the desired virtual repelant point for the i-th step.
  //        t_step    - the time interval to use for backwards integration
  //        dcm_eos_i - the DCM state at the end of the i-th step. 
  // computes the step i's initial DCM state and the end-of-step i-1's dcm state. 
  // The computation is stored in the dcm_ini_list and dcm_eos_list. 
  void computeDCM_ini_i(const Eigen::Vector3d & r_vrp_d_i, const double & t_step, const Eigen::Vector3d & dcm_eos_i);

  // computes all the dcm states. Computation properly populates the dcm_ini_list and dcm_eos_list
  void computeDCM_states();

};

#endif