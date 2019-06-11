#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>
#include <avatar_locomanipulation/data_types/footstep.hpp>

#include <avatar_locomanipulation/helpers/hermite_curve.hpp>

// Standard
#include <iostream>
#include <math.h>

void printQuat(const Eigen::Quaternion<double> & quat){
  std::cout << "quaternion:" << quat.x() << " " <<
                                  quat.y() << " " <<
                                  quat.z() << " " <<
                                  quat.w() << " " << std::endl;
}

int main(int argc, char ** argv){
  WalkingPatternGenerator wpg;
  Footstep footstep_object_test;

  // double t = 1.0;
  // double omega_val = M_PI/4.0;
  // Eigen::AngleAxis<double> omega(omega_val*t, Eigen::Vector3d(0.0, 1.0, 0.0));
  // Eigen::Quaternion<double> omega_quat(0.0, 0.0, omega_val, 0.0);

  // Eigen::Quaternion<double> quat(omega);
  // printQuat(quat);

  // Eigen::Quaternion<double> quat_dot = quat*omega_quat;  

  // Eigen::Quaternion<double> reobtain_omega_quat = quat.inverse()*quat_dot;
  // printQuat(reobtain_omega_quat);

  HermiteCurve h1(0, 0, 8, 0);
  std::cout << "evaluate h1(t=0.0) = " << h1.evaluate(0.0) << std::endl; 
  std::cout << "evaluate dh1(t=0.0) = " << h1.evaluateFirstDerivative(0.0) << std::endl; 
  std::cout << "evaluate ddh1(t=0.0) = " << h1.evaluateSecondDerivative(0.0) << std::endl; 

  std::cout << "evaluate h1(t=1.0) = " << h1.evaluate(1.0) << std::endl; 
  std::cout << "evaluate dh1(t=1.0) = " << h1.evaluateFirstDerivative(1.0) << std::endl; 
  std::cout << "evaluate ddh1(t=1.0) = " << h1.evaluateSecondDerivative(1.0) << std::endl; 

  // int N = 1000;
  // double dt = 1.0/((double) N);
  // double t = 0.0;
  // std::cout << dt << std::endl;
  // std::cout << "t," << "h(t)," << "dh(t)," << "dd(ht)" << std::endl; 
  // for(int i = 0; i < (N+1); i++){
  //   t = i*dt;
  //   std::cout << t << "," << h1.evaluate(t) << "," << h1.evaluateFirstDerivative(t) << "," << h1.evaluateSecondDerivative(t) << std::endl;    
  // }

  return 0;
}