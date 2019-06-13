#ifndef FEASIBILITY_H
#define FEASIBILITY_H
// Standard
#include <math.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
// Include Punocchio Related Libraries
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
// Articulated Body Algorithm
#include "pinocchio/algorithm/aba.hpp"
// Composite Rigid Body Algorithm
#include "pinocchio/algorithm/crba.hpp"
// Recursive Newton-Euler Algorithm
#include "pinocchio/algorithm/rnea.hpp"
//
// #include <math.h>
#include <avatar_locomanipulation/models/valkyrie_model.hpp>

#define SVD_SOLVER JacobiSVD

class feasibility{
public:
  feasibility();
  ~feasibility();
  //Initialize Valkyrie Model
  ValkyrieModel valkyrie;

  Eigen::VectorXd q_start;
  Eigen::VectorXd q_end;
  Eigen::VectorXd dq_change;

  // Right foot will projected to origin, facing forward
  // That way the global frame is equivalent to the right foot frame
  Eigen::Vector3d rfoot_des_pos;
  Eigen::Quaternion<double> rfoot_des_quat;
  Eigen::Vector3d rfoot_pos_error;
  Eigen::Vector3d rfoot_ori_error;

  Eigen::Vector3d rfoot_cur_pos;
  Eigen::Quaternion<double> rfoot_cur_ori;
  Eigen::MatrixXd J_rfoot;

  // Left foot projected to ground, no specification on its orientation
  Eigen::Vector3d lfoot_des_pos;
  Eigen::Quaternion<double> lfoot_des_quat;
  Eigen::Vector3d lfoot_pos_error;
  Eigen::Vector3d lfoot_ori_error;

  Eigen::Vector3d lfoot_cur_pos;
  Eigen::Quaternion<double> lfoot_cur_ori;
  Eigen::MatrixXd J_lfoot;


  // CoM in support region and specified height
  Eigen::Vector3d com_des_pos;
  Eigen::Vector3d com_pos_error;

  // This is the fully stacked jacobian
  Eigen::MatrixXd J_task;
  Eigen::VectorXd task_error;

  Eigen::VectorXd upper_lim;
  Eigen::VectorXd lower_lim;

  double ik_error_norm = 1000.0;

  // SVD_SOLVER
  std::unique_ptr< Eigen::SVD_SOLVER<Eigen::MatrixXd> > svd;
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;

  void ComputeTransError(const Eigen::Vector3d & des, const Eigen::Vector3d & current, Eigen::Vector3d & error);

  void ComputeQuatError(const Eigen::Quaternion<double> & des,
                    const Eigen::Quaternion<double> & current,
                    Eigen::Vector3d & error);

  void InverseKinematicsTop(const int & maxsteps);
  //void feasibility::InverseKinematicsBot(int & maxsteps);

  void initialize_configuration_top();
  void initialize_desired_top();
  void initialize_upper_limits();
  void initialize_lower_limits();
  double clamp(const double & s_in, double lo, double hi);

private:
  Eigen::AngleAxis<double> axis_angle;


};

#endif
