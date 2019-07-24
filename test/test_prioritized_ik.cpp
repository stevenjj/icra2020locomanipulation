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
// Robot Model
#include <avatar_locomanipulation/models/robot_model.hpp>
// PseudoInverse
#include <avatar_locomanipulation/helpers/pseudo_inverse.hpp>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

void testIK(Eigen::VectorXd & q_init, Eigen::VectorXd & q_final);

int main(int argc, char ** argv){
  Eigen::VectorXd q_start;
  Eigen::VectorXd q_end;
  testIK(q_start, q_end);


  // Initialize ROS node for publishing joint messages
  ros::init(argc, argv, "test_rviz");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);

  // Initialize Rviz translator
  ValRvizTranslator rviz_translator;

  // Transform broadcaster
  tf::TransformBroadcaster      br_ik;
  tf::TransformBroadcaster      br_robot;
  // Joint State Publisher
  ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
  ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);

  // Initialize Transforms and Messages
  tf::Transform tf_world_pelvis_init;
  tf::Transform tf_world_pelvis_end;

  sensor_msgs::JointState joint_msg_init;
  sensor_msgs::JointState joint_msg_end;

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

  RobotModel valkyrie(filename, meshDir, srdf_filename);
  // Visualize q_start and q_end in RVIZ
  rviz_translator.populate_joint_state_msg(valkyrie.model, q_start, tf_world_pelvis_init, joint_msg_init);
  rviz_translator.populate_joint_state_msg(valkyrie.model, q_end, tf_world_pelvis_end, joint_msg_end);

  while (ros::ok()){
      br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
      robot_joint_state_pub.publish(joint_msg_init);

      br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
      robot_ik_joint_state_pub.publish(joint_msg_end);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;

}



void testIK(Eigen::VectorXd & q_init, Eigen::VectorXd & q_final){
  // Test to see if this is working
  std::cout << "Initialize Valkyrie Model" << std::endl;
  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

  RobotModel valkyrie(filename, meshDir, srdf_filename);

  // X dimensional state vectors
  Eigen::VectorXd q_start;
  Eigen::VectorXd q_end;
  Eigen::VectorXd dq_change;

  // Setting configs to 0???
  q_start = Eigen::VectorXd::Zero(valkyrie.getDimQ());
  q_end = Eigen::VectorXd::Zero(valkyrie.getDimQ());
  dq_change = Eigen::VectorXd::Zero(valkyrie.getDimQdot());

  double theta = 0.0;//M_PI/4.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q

  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[valkyrie.getJointIndex("leftHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;

  q_start[valkyrie.getJointIndex("rightShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie.getJointIndex("rightElbowPitch")] = 0.4;
  q_start[valkyrie.getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;
  q_end = q_start;

  valkyrie.updateFullKinematics(q_end);

  // Set desired
  Eigen::Vector3d rfoot_des_pos;
  Eigen::Quaternion<double> rfoot_des_quat;
  Eigen::Vector3d rfoot_pos_error;
  Eigen::Vector3d rfoot_ori_error;

  Eigen::Vector3d rfoot_cur_pos;
  Eigen::Quaternion<double> rfoot_cur_ori;
  Eigen::MatrixXd J_rfoot;

  Eigen::Vector3d lfoot_des_pos;
  Eigen::Quaternion<double> lfoot_des_quat;
  Eigen::Vector3d lfoot_pos_error;
  Eigen::Vector3d lfoot_ori_error;

  Eigen::Vector3d lfoot_cur_pos;
  Eigen::Quaternion<double> lfoot_cur_ori;
  Eigen::MatrixXd J_lfoot;

  Eigen::MatrixXd J_task;
  Eigen::VectorXd task_error;

  // Foot should be flat on the ground and spaced out by 0.25m on each side (0,0,0)
  rfoot_des_pos.setZero();
  rfoot_des_pos[1] = -0.125;
  rfoot_des_quat.setIdentity();
  rfoot_pos_error.setZero();
  rfoot_ori_error.setZero();

  rfoot_cur_pos.setZero();
  rfoot_cur_ori.setIdentity();
  J_rfoot = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());

  lfoot_des_pos.setZero();
  lfoot_des_pos[1] = 0.125;
  lfoot_des_quat.setIdentity();
  lfoot_pos_error.setZero();
  lfoot_ori_error.setZero();

  lfoot_cur_pos.setZero();
  lfoot_cur_ori.setIdentity();
  J_lfoot = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());

  int task_dim = J_rfoot.rows() + J_lfoot.rows();
  J_task = Eigen::MatrixXd::Zero(task_dim, valkyrie.getDimQdot());
  task_error = Eigen::VectorXd::Zero(task_dim);

  std::unique_ptr< Eigen::JacobiSVD<Eigen::MatrixXd> > svd;
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;
  svd = std::unique_ptr< Eigen::JacobiSVD<Eigen::MatrixXd> >( new Eigen::JacobiSVD<Eigen::MatrixXd>(J_task.rows(), valkyrie.getDimQdot(), svdOptions) );
  const double svd_thresh = 1e-4;
  svd->setThreshold(svd_thresh);

  // IK Portion
  //Current Position and Orientation
  double ik_error_norm = 1000.0;
  double eps = 1e-6;

  Eigen::AngleAxis<double> axis_angle;

  for(int i=0; i<10; i++){

    valkyrie.updateFullKinematics(q_end);
    valkyrie.getFrameWorldPose("rightCOP_Frame", rfoot_cur_pos, rfoot_cur_ori);
    valkyrie.getFrameWorldPose("leftCOP_Frame", lfoot_cur_pos, lfoot_cur_ori);

    // std::cout << "rfoot ori:" << lfoot_cur_pos.transpose() << std::endl;
    // std::cout << "Quat (x,y,z,w): " << lfoot_cur_ori.x() << " " <<
    //                 lfoot_cur_ori.y() << " " <<
    //                 lfoot_cur_ori.z() << " " <<
    //                 lfoot_cur_ori.w() << " " <<
    // std::endl;

    rfoot_pos_error = rfoot_des_pos - rfoot_cur_pos;
    lfoot_pos_error = lfoot_des_pos - lfoot_cur_pos;
    axis_angle = rfoot_des_quat*rfoot_cur_ori.inverse();
    rfoot_ori_error = axis_angle.axis() * axis_angle.angle();
    axis_angle = lfoot_des_quat*lfoot_cur_ori.inverse();
    lfoot_ori_error = axis_angle.axis() * axis_angle.angle();

    task_error.head<3>() = rfoot_pos_error;
    task_error.segment<3>(3) = rfoot_ori_error;
    task_error.segment<3>(6) = lfoot_pos_error;
    task_error.segment<3>(9) = lfoot_ori_error;

    ik_error_norm = task_error.norm();

    valkyrie.get6DTaskJacobian("rightCOP_Frame", J_rfoot);
    valkyrie.get6DTaskJacobian("leftCOP_Frame", J_lfoot);

    J_task.topRows(6) = J_rfoot;
    J_task.bottomRows(6) = J_lfoot;

    dq_change = svd->compute(J_task).solve(task_error);
    valkyrie.forwardIntegrate(q_end, dq_change, q_end);

   std::cout << i << " error norm = " << ik_error_norm << std::endl;

  }

  std::cout << "Final error norm = " << ik_error_norm << std::endl;

 // output intermediate solution:
  q_init = q_end;


  // Do another IK. This time with priority
  double singular_values_threshold = 1e-4;

  int task_one_dim = J_rfoot.rows() + J_lfoot.rows();
  Eigen::MatrixXd J1_task = Eigen::MatrixXd::Zero(task_one_dim, valkyrie.getDimQdot());
  Eigen::MatrixXd J1pinv = Eigen::MatrixXd::Zero(valkyrie.getDimQdot(), task_one_dim);
  Eigen::VectorXd dx1 = Eigen::VectorXd::Zero(task_one_dim);
  std::unique_ptr< Eigen::JacobiSVD<Eigen::MatrixXd> > svd_J1;
  svd_J1 = std::unique_ptr< Eigen::JacobiSVD<Eigen::MatrixXd> >( new Eigen::JacobiSVD<Eigen::MatrixXd>(J_task.rows(), valkyrie.getDimQdot(), svdOptions) );
  // Joint Configuration change due to task 1
  Eigen::VectorXd dq1 = Eigen::VectorXd::Zero(valkyrie.getDimQdot());

  // initialize nullspace object
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(valkyrie.getDimQdot(), valkyrie.getDimQdot());
  Eigen::MatrixXd N1 = Eigen::MatrixXd::Zero(valkyrie.getDimQdot(), valkyrie.getDimQdot());

  // Add a lower priority right palm task
  Eigen::Vector3d rpalm_cur_pos;
  Eigen::Quaternion<double> rpalm_cur_ori;
  Eigen::Vector3d rpalm_pos_error;
  Eigen::Vector3d rpalm_ori_error;
  Eigen::MatrixXd J_rpalm = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());;

  // Set Desired to current configuration except right palm must have identity orientation
  Eigen::Vector3d rpalm_des_pos;
  Eigen::Quaternion<double> rpalm_des_quat;
  valkyrie.getFrameWorldPose("rightPalm", rpalm_des_pos, rpalm_des_quat);
  rpalm_des_pos[0] += 0.25; 
  rpalm_des_pos[1] += 0.25; 
  rpalm_des_pos[2] += 0.1; 
  axis_angle.angle() = (M_PI/2.0);
  axis_angle.axis() = Eigen::Vector3d(0, 0, 1.0);
  rpalm_des_quat = axis_angle;


  // Prepare jacobians 
  int task_two_dim = J_rpalm.rows();
  Eigen::MatrixXd J2_task = J_rpalm;
  Eigen::MatrixXd J2N1 = Eigen::MatrixXd::Zero(valkyrie.getDimQdot(), task_two_dim);
  Eigen::MatrixXd J2N1pinv = Eigen::MatrixXd::Zero(task_two_dim, valkyrie.getDimQdot());
  Eigen::VectorXd dx2 = Eigen::VectorXd::Zero(task_two_dim);
  std::unique_ptr< Eigen::JacobiSVD<Eigen::MatrixXd> > svd_J2;
  svd_J2 = std::unique_ptr< Eigen::JacobiSVD<Eigen::MatrixXd> >( new Eigen::JacobiSVD<Eigen::MatrixXd>(J2N1.rows(), valkyrie.getDimQdot(), svdOptions) );
  // Joint Configuration change due to task 2
  Eigen::VectorXd dq2 = Eigen::VectorXd::Zero(valkyrie.getDimQdot());
  Eigen::VectorXd total_error = Eigen::VectorXd::Zero(task_one_dim + task_two_dim);

  for(int i=0; i<100; i++){
    valkyrie.updateFullKinematics(q_end);
    valkyrie.getFrameWorldPose("rightCOP_Frame", rfoot_cur_pos, rfoot_cur_ori);
    valkyrie.getFrameWorldPose("leftCOP_Frame", lfoot_cur_pos, lfoot_cur_ori);
    valkyrie.getFrameWorldPose("rightPalm", rpalm_cur_pos, rpalm_cur_ori);

    valkyrie.get6DTaskJacobian("rightCOP_Frame", J_rfoot);
    valkyrie.get6DTaskJacobian("leftCOP_Frame", J_lfoot);
    valkyrie.get6DTaskJacobian("rightPalm", J_rpalm);

    // Task 1 errors
    rfoot_pos_error = rfoot_des_pos - rfoot_cur_pos;
    lfoot_pos_error = lfoot_des_pos - lfoot_cur_pos;
    axis_angle = rfoot_des_quat*rfoot_cur_ori.inverse();
    rfoot_ori_error = axis_angle.axis() * axis_angle.angle();
    axis_angle = lfoot_des_quat*lfoot_cur_ori.inverse();
    lfoot_ori_error = axis_angle.axis() * axis_angle.angle();
    // Stack Task 1 errors:
    dx1.head<3>() = rfoot_pos_error;
    dx1.segment<3>(3) = rfoot_ori_error;
    dx1.segment<3>(6) = lfoot_pos_error;
    dx1.segment<3>(9) = lfoot_ori_error;    
    // Stack Task 1 Jacobians
    J1_task.topRows(6) = J_rfoot;
    J1_task.bottomRows(6) = J_lfoot;

    // Compute Task 1:
    // Compute inertia matrix inverse
    valkyrie.computeInertiaMatrixInverse(q_end);

    math_utils::weightedPseudoInverse(J1_task, valkyrie.Ainv, *svd_J1, J1pinv, singular_values_threshold);
   // math_utils::pseudoInverse(J1_task, *svd_J1, J1pinv, singular_values_threshold, Eigen::ComputeThinU | Eigen::ComputeThinV);
    dq1 = J1pinv*dx1;
    // Compute Task 1 Nullspace
    N1 = (I - J1pinv*J1_task);

    // Task 2 errors
    rpalm_pos_error = rpalm_des_pos - rpalm_cur_pos;
    axis_angle = rpalm_des_quat*rpalm_cur_ori.inverse();
    rpalm_ori_error = axis_angle.axis() * axis_angle.angle();
    // Stack Task 2 errors:
    dx2.head<3>() = rpalm_pos_error;
    dx2.tail<3>() = rpalm_ori_error;  
    // Stack Task 2 Jacobians
    J2_task = J_rpalm;

    // Compute Task 2
    J2N1 = J2_task*N1;
    math_utils::weightedPseudoInverse(J2N1, valkyrie.Ainv, *svd_J2, J2N1pinv, singular_values_threshold);
    // math_utils::pseudoInverse(J2N1, *svd_J2, J2N1pinv, singular_values_threshold, Eigen::ComputeThinU | Eigen::ComputeThinV);
    dq2 = J2N1pinv*(dx2 - J2_task*dq1);

    // Compute total error
    total_error.head(dx1.rows()) = dx1;
    total_error.tail(dx2.rows()) = dx2;

    // Forward Integrate
    dq_change = dq1 + dq2;
    valkyrie.forwardIntegrate(q_end, 0.1*dq_change, q_end);

    std::cout << "errors = norm(dx1), norm(dx2), norm([dx1^T,dx2^T]) =  " << dx1.norm() << ", " << dx2.norm() << ", " << total_error.norm() << std::endl;
  }

 
  // Output final solution
  q_final = q_end;  

}