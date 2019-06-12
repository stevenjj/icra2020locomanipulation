// Standard
#include <math.h>
#include <iostream>
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
  double lfoot_des_pos = 0;
  double lfoot_pos_error;

  Eigen::Vector3d lfoot_cur_pos;
  Eigen::Quaternion<double> lfoot_cur_ori;
  Eigen::MatrixXd J_lfoot;

  // CoM in support region and specified height
  double CoM_des_z = 2;

  Eigen::MatrixXd J_CoM;


  // This is the fully stacked jacobian
  Eigen::MatrixXd J_task;
  Eigen::VectorXd task_error;

  double ik_error_norm = 1000.0;

  // SVD_SOLVER
  std::unique_ptr< Eigen::SVD_SOLVER<Eigen::MatrixXd> > svd;

  void feasibility::ComputeTransError(const Eigen::Vector3d & des, const Eigen::Vector3d & current, Eigen::Vector3d & error);

  void feasibility::ComputeQuatError(const Eigen::Quaternion<double> & des,
      							const Eigen::Quaternion<double> & current,
      							Eigen::Vector3d & error);
}
