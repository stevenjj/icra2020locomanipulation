// // Standard
// #include <math.h>
// #include <iostream>
// // Include Punocchio Related Libraries
// #include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h>
//
// #include "pinocchio/multibody/model.hpp"
// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/jacobian.hpp"
// #include "pinocchio/algorithm/frames.hpp"
// // Articulated Body Algorithm
// #include "pinocchio/algorithm/aba.hpp"
// // Composite Rigid Body Algorithm
// #include "pinocchio/algorithm/crba.hpp"
// // Recursive Newton-Euler Algorithm
// #include "pinocchio/algorithm/rnea.hpp"
// //
// // #include <math.h>
// #include <avatar_locomanipulation/models/valkyrie_model.hpp>

// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/valkyrie_model.hpp>

#include <avatar_locomanipulation/feasibility/feasibility.hpp>

#define SVD_SOLVER JacobiSVD

int main(int argc, char ** argv){
    // std::cout <<RAND_MAX << std::endl;

    // feasibility fs;
    // fs.InverseKinematicsTop(10);
  // Test to see if this is working
  // std::cout << "Initialize Valkyrie Model" << std::endl;
  ValkyrieModel valkyrie;
  //
  // // X dimensional state vectors
  // Eigen::VectorXd q_start;
  // Eigen::VectorXd q_end;
  // Eigen::VectorXd dq_change;
  //
  // // Setting configs to 0???
	// q_start = Eigen::VectorXd::Zero(valkyrie.getDimQ());
	// q_end = Eigen::VectorXd::Zero(valkyrie.getDimQ());
	// dq_change = Eigen::VectorXd::Zero(valkyrie.getDimQdot());
  //  srand(time(0));
  // double theta = M_PI/4.0;
	// Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
  //
  // Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  // init_quat = aa;
  //
  // q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q
  //
  // q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location
  //
  // q_start[valkyrie.getJointIndex("leftHipPitch")] = -0.3;
  // q_start[valkyrie.getJointIndex("rightHipPitch")] = -0.3;
  // q_start[valkyrie.getJointIndex("leftKneePitch")] = 0.6;
  // q_start[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
  // q_start[valkyrie.getJointIndex("leftAnklePitch")] = -0.3;
  // q_start[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;
  //
  // q_start[valkyrie.getJointIndex("rightShoulderPitch")] = -0.2;
  // q_start[valkyrie.getJointIndex("rightShoulderRoll")] = 1.1;
  // q_start[valkyrie.getJointIndex("rightElbowPitch")] = 0.4;
  // q_start[valkyrie.getJointIndex("rightForearmYaw")] = 1.5;
  //
  // q_start[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2;
  // q_start[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;
  // q_start[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
  // q_start[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;
  // q_end = q_start;
  //
  // valkyrie.updateFullKinematics(q_end);
  //
  // // Set desired
  // Eigen::Vector3d rfoot_des_pos;
  // Eigen::Quaternion<double> rfoot_des_quat;
  // Eigen::Vector3d rfoot_pos_error;
  // Eigen::Vector3d rfoot_ori_error;
  //
  // Eigen::Vector3d rfoot_cur_pos;
	// Eigen::Quaternion<double> rfoot_cur_ori;
	// Eigen::MatrixXd J_rfoot;
  //
  // Eigen::Vector3d lfoot_des_pos;
  // Eigen::Quaternion<double> lfoot_des_quat;
  // Eigen::Vector3d lfoot_pos_error;
  // Eigen::Vector3d lfoot_ori_error;
  //
  // Eigen::Vector3d lfoot_cur_pos;
  // Eigen::Quaternion<double> lfoot_cur_ori;
  // Eigen::MatrixXd J_lfoot;
  //
  // Eigen::MatrixXd J_task;
  // Eigen::VectorXd task_error;
  //
  // // Foot should be flat on the ground and spaced out by 0.25m on each side (0,0,0)
  // rfoot_des_pos.setZero();
  // rfoot_des_pos[1] = -0.125;
  // rfoot_des_quat.setIdentity();
  // rfoot_pos_error.setZero();
  // rfoot_ori_error.setZero();
  //
  // rfoot_cur_pos.setZero();
  // rfoot_cur_ori.setIdentity();
  // J_rfoot = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());
  //
  // lfoot_des_pos.setZero();
  // lfoot_des_pos[1] = 0.125;
  // lfoot_des_quat.setIdentity();
  // lfoot_pos_error.setZero();
  // lfoot_ori_error.setZero();
  //
  // lfoot_cur_pos.setZero();
  // lfoot_cur_ori.setIdentity();
  // J_lfoot = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());
  //
  // int task_dim = J_rfoot.rows() + J_lfoot.rows();
  // J_task = Eigen::MatrixXd::Zero(task_dim, valkyrie.getDimQdot());
  // task_error = Eigen::VectorXd::Zero(task_dim);
  //
  // std::unique_ptr< Eigen::SVD_SOLVER<Eigen::MatrixXd> > svd;
  // unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;
  // svd = std::unique_ptr< Eigen::SVD_SOLVER<Eigen::MatrixXd> >( new Eigen::SVD_SOLVER<Eigen::MatrixXd>(J_task.rows(), valkyrie.getDimQdot(), svdOptions) );
  // const double svd_thresh = 1e-4;
  // svd->setThreshold(svd_thresh);
  //
  // // IK Portion
  // //Current Position and Orientation
  // double ik_error_norm = 1000.0;
  // double eps = 1e-6;
  //
  // Eigen::AngleAxis<double> axis_angle;
  //
  // for(int i=0; i<10; i++){
  //
  //   valkyrie.updateFullKinematics(q_end);
  //   valkyrie.getFrameWorldPose("rightCOP_Frame", rfoot_cur_pos, rfoot_cur_ori);
  //   valkyrie.getFrameWorldPose("leftCOP_Frame", lfoot_cur_pos, lfoot_cur_ori);
  //
  //   // std::cout << "rfoot ori:" << lfoot_cur_pos.transpose() << std::endl;
  //   // std::cout << "Quat (x,y,z,w): " << lfoot_cur_ori.x() << " " <<
  // 	// 								 lfoot_cur_ori.y() << " " <<
  // 	// 								 lfoot_cur_ori.z() << " " <<
  // 	// 								 lfoot_cur_ori.w() << " " <<
  //   // std::endl;
  //
  //   rfoot_pos_error = rfoot_des_pos - rfoot_cur_pos;
  //   lfoot_pos_error = lfoot_des_pos - lfoot_cur_pos;
  //   axis_angle = rfoot_des_quat*rfoot_cur_ori.inverse();
  //   rfoot_ori_error = axis_angle.axis() * axis_angle.angle();
  //   axis_angle = lfoot_des_quat*lfoot_cur_ori.inverse();
  //   lfoot_ori_error = axis_angle.axis() * axis_angle.angle();
  //
  //   task_error.head<3>() = rfoot_pos_error;
  //   task_error.segment<3>(3) = rfoot_ori_error;
  //   task_error.segment<3>(6) = lfoot_pos_error;
  //   task_error.segment<3>(9) = lfoot_ori_error;
  //
  //   ik_error_norm = task_error.norm();
  //
  //   valkyrie.get6DTaskJacobian("rightCOP_Frame", J_rfoot);
  //   valkyrie.get6DTaskJacobian("leftCOP_Frame", J_lfoot);
  //
  //   J_task.topRows(6) = J_rfoot;
  //   J_task.bottomRows(6) = J_lfoot;
  //
  //   dq_change = svd->compute(J_task).solve(task_error);
  //   valkyrie.forwardIntegrate(q_end, dq_change, q_end);
  //
  //  std::cout << i << " error norm = " << ik_error_norm << std::endl;
  //
  // }
  //
  // std::cout << "Final error norm = " << ik_error_norm << std::endl;
 valkyrie.printFrameNames();
  return 0;
}




// add_executable(test_mihir_working_file test_mihir_working_file.cpp ${PROJECT_SOURCES})
// target_link_libraries(test_mihir_working_file ${catkin_LIBRARIES})

//rosrun avatar_locomanipulation test_mihir_working_file
