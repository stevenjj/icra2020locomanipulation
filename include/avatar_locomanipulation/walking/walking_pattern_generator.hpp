#ifndef WALKING_PATTERN_GENERATOR_H
#define WALKING_PATTERN_GENERATOR_H

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <avatar_locomanipulation/data_types/footstep.hpp>
#include <avatar_locomanipulation/helpers/hermite_curve.hpp>
#include <avatar_locomanipulation/helpers/hermite_quaternion_curve.hpp>
#include <iostream>


class WalkingPatternGenerator{
public:
  WalkingPatternGenerator();
  ~WalkingPatternGenerator(); 

  static int const SWING_VRP_TYPE;
  static int const DOUBLE_SUPPORT_TRANSFER_VRP_TYPE;

  std::vector<Footstep> footstep_list;

  std::vector<Eigen::Vector3d> rvrp_list; // List of virtual repelant points.
  std::vector<Eigen::Vector3d> dcm_ini_list; // List of initial DCM states 
  std::vector<Eigen::Vector3d> dcm_eos_list; // List of end-of-step DCM states

  std::vector<int> rvrp_type_list; // List of type of virtual repelant point


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

  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        left_footstance        - a footstep object describing the left stance feet
  //        right_footstance       - a footstep object describing the right stance feet
  // populates this object's footstep_list, rvrp_list, dcm_ini_list, dcm_eos_list. 
  void initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, const Footstep & left_footstance, const Footstep & right_footstance);

   // Outputs the average r_vrp location given two footstances
  void get_average_rvrp(const Footstep & footstance_1, const Footstep & footstance_2, Eigen::Vector3d & average_rvrp);

  // Swing trajectory calculation
  void computeSE3_trajectory(const Footstep & init_stance_location, const Footstep & landing_location);

  // computes all the dcm states. Computation properly populates the dcm_ini_list and dcm_eos_list
  void computeDCM_states();

  // Initialize trajectory clock
  void initialize_internal_clocks();

  // Given a delta_t compute the next desired DCM
  Eigen::Vector3d get_next_desired_DCM(const double & dt);

  double get_total_trajectory_time();
  void initialize_trajectory_discretization(const int & N_samples);
  void construct_trajectories();



private:
  // input: r_vrp_d_i - the desired virtual repelant point for the i-th step.
  //        t_step    - the time interval to use for backwards integration
  //        dcm_eos_i - the DCM state at the end of the i-th step. 
  // computes the step i's initial DCM state and the end-of-step i-1's dcm state. 
  // The computation is stored in the dcm_ini_list and dcm_eos_list. 
  Eigen::Vector3d computeDCM_ini_i(const Eigen::Vector3d & r_vrp_d_i, const double & t_step, const Eigen::Vector3d & dcm_eos_i);

  // Get the t_step for step i.
  double get_t_step(const int & step_i);

  double internal_timer;
  double internal_t_step;
  double internal_step_timer;
  int internal_step_i;

  Footstep mid_foot_;

};

#endif