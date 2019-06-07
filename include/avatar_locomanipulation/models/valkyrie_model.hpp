#ifndef ALM_VALKYRIE_MODEL_H
#define ALM_VALKYRIE_MODEL_H

#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Multibody
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
// Algorithms
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp" // Jacobian Frame Computation 
#include "pinocchio/algorithm/aba.hpp" // Articulated Body Algorithm
#include "pinocchio/algorithm/crba.hpp" // Composite Rigid Body Algorithm
#include "pinocchio/algorithm/rnea.hpp" // Recursive Newton-Euler Algorithm
#include "pinocchio/algorithm/center-of-mass.hpp" // Center-of-Mass related algorithms
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp" 
// Parsers
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

// Utils
#include "pinocchio/utils/timer.hpp"

// Standard
#include <math.h>

// Macros
#define VAL_MODEL_NUM_FLOATING_JOINTS 7 // 3 for x,y,z and 4 for qx, qy, qz, qw
#define VAL_MODEL_JOINT_INDX_OFFSET 2 //pinocchio attaches a universe joint and a root joint that we need to remove


class ValkyrieModel{
public:
  ValkyrieModel();
  ValkyrieModel(const std::string & filename);

  ~ValkyrieModel();

  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;

  Eigen::MatrixXd A; // inertia matrix
  Eigen::MatrixXd Ainv; // inverse of the inertia matrix
  Eigen::MatrixXd C; // coriolis matrix
  Eigen::VectorXd g; // gravity vector

  Eigen::Vector3d x_com; // com position
  Eigen::Vector3d xdot_com; // com velocity
  Eigen::Vector3d xddot_com; // com acceleration

  Eigen::Matrix3Xd J_com; // Center of mass Jacobian
  Eigen::Matrix3Xd Jdot_com; // Center of mass Jacobian Derivative


  /* updateFullKinematics
  Input: a vector of configuration with dimension model.nq to update the kinematics.   
  */
  void updateFullKinematics(const Eigen::VectorXd & q_update);

  // updates the kinematic derivatives
  void updateKinematicsDerivatives(const Eigen::VectorXd & q_update, const Eigen::VectorXd & qdot_update, const Eigen::VectorXd & qddot_update);

  // updates the 
  void updateJointJacobiansDerivatives(const Eigen::VectorXd & q_update, const Eigen::VectorXd & qdot_update);

  /* get6DTaskJacobian
  Input: the frame name.   
  Output: the 6D task jacobian (dimension 6 x model.nv) expressed in world frame
          with the linear components first: [(dx/dq)^T , (dy/dq)^T, (dz/dq)^T]^T
          then the rotational components: [(wx/dq)^T , (wy/dq)^T, (wz/dq)^T]^T
  */
  void get6DTaskJacobian(const std::string & frame_name, Eigen::MatrixXd & J_out);

  /* get6DTaskJacobianDot
  Input: the frame name.
  Output: the 6D task jacobian dot (6 x model.nv) expressed in world frame.
  */
  void get6DTaskJacobianDot(const std::string & frame_name, Eigen::MatrixXd & Jdot_out);


  /* getFrameWorldPose
  Input: the frame name.   
  Output: the position and orientation of the frame with respect to world.
  */

  void getFrameWorldPose(const std::string & name, Eigen::Vector3d & pos, Eigen::Quaternion<double> & ori);

  /* getJointIndex
  Input std::string name
  Output: The joint index in configuration space
      For Valkyrie, the first 7 joints are floating base joints. 3 for the linear and 4 for the rotation described as a quaternion.       
      Per C++ standard, the index starts at 0.

      Thus, for a "leftHipYaw" input, which is the first joint in the kinematic chain,
      this function should return 7.
  */
  int getJointIndex(const std::string & name); 

  /* getDimQ();
  Output: The dimension of the configuration space
  */
  int getDimQ();
  /* getDimQdot();
  Output: The dimension of the configuration space
  */
  int getDimQdot();

  // Tangent space forward integration given q_start, qdot*dt 
  void forwardIntegrate(const Eigen::VectorXd & q_start, const Eigen::VectorXd & qdotDt, Eigen::VectorXd & q_post);

  // Computes the inertia matrix dim(model.nv x model.nv) given configuration and stores the result.
  void computeInertiaMatrix(const Eigen::VectorXd & q);
  // Computes the inverse of the inertia matrix dim(model.nv x model.nv) given configuration and stores the result.
  void computeInertiaMatrixInverse(const Eigen::VectorXd & q);
  // Computes the coriolis matrix dim(model.nv x model.nv) given configuration and velocity and stores the result.
  void computeCoriolisMatrix(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot);
  // Computes the gravity vector dim(model.nv) given configuration and stores the result
  void computeGravityVector(const Eigen::VectorXd & q);


  // CoM positions, velocities, and accelerations are expressed in world frame.
  // Computes the CoM position given q
  void computeCoMPos(const Eigen::MatrixBase<Eigen::VectorXd> & q);
  // Computes the CoM position and velocity given q and qdot
  void computeCoMPosVel(const Eigen::MatrixBase<Eigen::VectorXd> & q, const Eigen::MatrixBase<Eigen::VectorXd> & qdot);
  // Computes the CoM position, velocity, and acceleration given q, qdot, and qddot
  void computeCoMPosVelAcc(const Eigen::MatrixBase<Eigen::VectorXd> & q, const Eigen::MatrixBase<Eigen::VectorXd> & qdot, const Eigen::MatrixBase<Eigen::VectorXd> & qddot);

  // Computes the Jacobian and the position of the Center of Mass of the robot expressed in world frame
  //  Assumes that pinocchio::forwardKinematics() has been called.
  //  the computation also runs a computeJointJacobians() routine. 
  //  value is stored in J_com;
  void computeCoMJacobian();  
  // Computes the Jacobian and the position of the Center of Mass of the robot expressed in world frame
  // the computation also runs a computeJointJacobians() routine. 
  // value is stored in J_com;
  void computeCoMJacobian(const Eigen::MatrixBase<Eigen::VectorXd> & q);  

  // Computes the value of Jdot_com. Assumes that updateKinematicsDerivatives was called.
  void computeCoMJacobianDot(const Eigen::MatrixBase<Eigen::VectorXd> & q, const Eigen::MatrixBase<Eigen::VectorXd> & qdot);  

  void printJointNames();
  void printFrameNames();

private:
  // private temporary index value holders. Prevents memory allocation on runtime
  pinocchio::FrameIndex tmp_frame_index;
  pinocchio::JointIndex tmp_joint_index;
  void buildPinocchioModel(const std::string & filename);
  void commonInitialization();
};

#endif 