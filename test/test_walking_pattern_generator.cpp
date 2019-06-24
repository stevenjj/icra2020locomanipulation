#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>
#include <avatar_locomanipulation/data_types/footstep.hpp>
#include <avatar_locomanipulation/data_types/trajectory_euclidean.hpp>

#include <avatar_locomanipulation/helpers/hermite_curve.hpp>
#include <avatar_locomanipulation/helpers/hermite_quaternion_curve.hpp>

// Standard
#include <iostream>
#include <math.h>
#include <cassert>

void printQuat(const Eigen::Quaternion<double> & quat){
  std::cout <<  quat.x() << " " <<
                quat.y() << " " <<
                quat.z() << " " <<
                quat.w() << " " << std::endl;
}

void testTrajectories(){
  WalkingPatternGenerator wpg;
  wpg.initialize_trajectory_discretization(1000);

  // Initialize footstep objects
  Footstep right_foot_stance; 
  right_foot_stance.robot_side = RIGHT_FOOTSTEP;
  
  Footstep left_foot_stance; 
  left_foot_stance.robot_side = LEFT_FOOTSTEP; 
  left_foot_stance.position[1] = 0.25; 

  // Take a left footstep forward
  Footstep step1; 
  step1.robot_side = LEFT_FOOTSTEP;
  step1 = left_foot_stance; step1.position[0] += 0.35;
  Footstep step2; 
  step2 = right_foot_stance; step2.position[0] = step1.position[0] + 0.35;
  step2.robot_side = RIGHT_FOOTSTEP;
  Footstep step3; 
  step3 = step1; step3.position[0] = step2.position[0] + 0.35;
  step3.robot_side = LEFT_FOOTSTEP;

  right_foot_stance.printInfo();
  left_foot_stance.printInfo();
  step1.printInfo();
  step2.printInfo();
  step3.printInfo();
  std::vector<Footstep> footstep_list = {step1, step2, step3, step3};


}

void dcm_test(){
  WalkingPatternGenerator wpg;

  // Initialize footstep objects
  Footstep right_foot_stance; 
  right_foot_stance.robot_side = RIGHT_FOOTSTEP;
  
  Footstep left_foot_stance; 
  left_foot_stance.robot_side = LEFT_FOOTSTEP; 
  left_foot_stance.position[1] = 0.25; 

  // Take a left footstep forward
  Footstep step1; 
  step1.robot_side = LEFT_FOOTSTEP;
  step1 = left_foot_stance; step1.position[0] += 0.35;
  Footstep step2; 
  step2 = right_foot_stance; step2.position[0] = step1.position[0] + 0.35;
  step2.robot_side = RIGHT_FOOTSTEP;
  Footstep step3; 
  step3 = step1; step3.position[0] = step2.position[0] + 0.35;
  step3.robot_side = LEFT_FOOTSTEP;

  right_foot_stance.printInfo();
  left_foot_stance.printInfo();
  step1.printInfo();
  step2.printInfo();
  step3.printInfo();

  std::vector<Footstep> footstep_list = {step1, step2, step3, step3};

  // Initialize rvrp given footstances
  wpg.initialize_footsteps_rvrp(footstep_list, left_foot_stance, right_foot_stance);    

  for(int i = 0; i < wpg.rvrp_list.size(); i++){
    std::cout << "i:" << i << " : " << wpg.rvrp_list[i].transpose() << std::endl;
  }

  wpg.computeDCM_states();

  std::cout << "initial DCM states:" << std::endl;
  for(int i = 0; i < wpg.dcm_ini_list.size(); i++){
    std::cout << "  i:" << i << " : " << wpg.dcm_ini_list[i].transpose() << std::endl;
  }

  std::cout << "end of step DCM states:" << std::endl;
  for(int i = 0; i < wpg.dcm_eos_list.size(); i++){
    std::cout << "  i:" << i << " : " << wpg.dcm_eos_list[i].transpose() << std::endl;
  }

  // test time evolution of Center-of-mass
  wpg.initialize_internal_clocks();

  Eigen::Vector3d x_com; x_com.setZero();
  Eigen::Vector3d zeta_dcm; zeta_dcm.setZero();

  double b = wpg.b;
  wpg.get_average_rvrp(left_foot_stance, right_foot_stance, x_com);

  std::cout << "starting x_com state" << std::endl;
  std::cout << x_com.transpose() << std::endl;

  double t = 0.0;
  double dt = 0.01;
  double total_sim_time = wpg.get_total_trajectory_time();
  int N_steps = (int)(total_sim_time/dt);

  // prepare trajectory vector
  TrajEuclidean com_traj(x_com.size(), N_steps, dt);
  TrajEuclidean dcm_traj(zeta_dcm.size(), N_steps, dt);

  // Compute data
  for(int i = 0; i < N_steps; i++){
    t = i*dt;
    if (i == 0){
      zeta_dcm = wpg.get_next_desired_DCM(0.0);
    }else{
      zeta_dcm = wpg.get_next_desired_DCM(dt);     
      x_com = (-1/b)*(x_com - zeta_dcm)*dt + x_com;
    }

    // Store the data
    com_traj.set_pos(i, x_com);
    dcm_traj.set_pos(i, zeta_dcm);

  }

  // retrieve the data:
  std::cout << "N=" << N_steps << std::endl;
  std::cout << "t, com_x, com_y, com_z, dcm_x_des, dcm_y_des, dcm_z_des" << std::endl;
  for(int i = 0; i < N_steps; i++){
    t = i*dt;
    com_traj.get_pos(i, x_com);
    dcm_traj.get_pos(i, zeta_dcm);
    std::cout << t << "," << x_com[0] << "," << x_com[1] << "," << x_com[2] << "," 
                          << zeta_dcm[0] << "," << zeta_dcm[1] << "," << zeta_dcm[2] << std::endl;
  
  }  

  wpg.computeSE3_trajectory(left_foot_stance, step1);  
}


void midfeet_test(){
  // Initialize footstep objects
  Footstep right_foot_stance; 
  right_foot_stance.robot_side = RIGHT_FOOTSTEP;
  
  Footstep left_foot_stance; 
  left_foot_stance.robot_side = LEFT_FOOTSTEP; 
  left_foot_stance.position[1] = 0.25; 


  Eigen::Vector3d omega(0, 0, 1);

  Eigen::AngleAxis<double> omega_aa(omega.norm(), omega/omega.norm());
  Footstep mid_stance;
  omega_aa.axis() = Eigen::Vector3d(0, 0, 1);
  omega_aa.angle() = M_PI/4.0;
  left_foot_stance.orientation = omega_aa;
  mid_stance.computeMidfeet(left_foot_stance, right_foot_stance, mid_stance);

  // Angle Axis
  Eigen::AngleAxisd aa_data;

  std::cout << "Test midfeet" << std::endl;
  std::cout << "Left Footstep:" << std::endl;
  left_foot_stance.printInfo();
  aa_data = left_foot_stance.orientation;
  std::cout << "angle: " << aa_data.angle() << " axis:" << aa_data.axis().transpose() << std::endl;

  std::cout << "Right Footstep:" << std::endl;
  right_foot_stance.printInfo();
  aa_data = right_foot_stance.orientation;
  std::cout << "angle: " << aa_data.angle() << " axis:" << aa_data.axis().transpose() << std::endl;

  std::cout << "Mid Footstep:" << std::endl;
  mid_stance.printInfo();
  aa_data = mid_stance.orientation;
  std::cout << "angle: " << aa_data.angle() << " axis:" << aa_data.axis().transpose() << std::endl;  
}

void hermite_curves_test(){
  // double t = 1.0;
  // double omega_val = M_PI/4.0;
  // Eigen::AngleAxis<double> omega(omega_val*t, Eigen::Vector3d(0.0, 1.0, 0.0));
  // Eigen::Quaternion<double> omega_quat(0.0, 0.0, omega_val, 0.0);

  // Eigen::Quaternion<double> quat(omega);
  // printQuat(quat);

  // Eigen::Quaternion<double> quat_dot = quat*omega_quat;  

  // Eigen::Quaternion<double> reobtain_omega_quat = quat.inverse()*quat_dot;
  // printQuat(reobtain_omega_quat);

  Eigen::Vector3d omega(10.5, 0.1, 0.3);

  Eigen::AngleAxis<double> omega_aa(omega.norm(), omega/omega.norm());
  std::cout << "omega axis angle = " << std::endl;
  std::cout << (omega_aa.axis()*omega_aa.angle()).transpose() << std::endl;
 
  Eigen::Quaterniond quat_omega(omega_aa);
  printQuat(quat_omega);

  HermiteCurve h1(0, 0, 8, 0);
  std::cout << "evaluate h1(t=0.0) = " << h1.evaluate(0.0) << std::endl; 
  std::cout << "evaluate dh1(t=0.0) = " << h1.evaluateFirstDerivative(0.0) << std::endl; 
  std::cout << "evaluate ddh1(t=0.0) = " << h1.evaluateSecondDerivative(0.0) << std::endl; 

  std::cout << "evaluate h1(t=1.0) = " << h1.evaluate(1.0) << std::endl; 
  std::cout << "evaluate dh1(t=1.0) = " << h1.evaluateFirstDerivative(1.0) << std::endl; 
  std::cout << "evaluate ddh1(t=1.0) = " << h1.evaluateSecondDerivative(1.0) << std::endl; 

  int N = 1000;
  double dt = 1.0/((double) N);
  double t = 0.0;
  // std::cout << dt << std::endl;
  // std::cout << "t," << "h(t)," << "dh(t)," << "dd(ht)" << std::endl; 
  // for(int i = 0; i < (N+1); i++){
  //   t = i*dt;
  //   std::cout << t << "," << h1.evaluate(t) << "," << h1.evaluateFirstDerivative(t) << "," << h1.evaluateSecondDerivative(t) << std::endl;    
  // }


  Eigen::Quaterniond qstart(1, 0, 0, 0);
  Eigen::AngleAxisd aa_end(M_PI/4.0, Eigen::Vector3d(0.5,1,2));
  Eigen::Quaterniond qend(aa_end); qend.normalize();
  // Eigen::Quaterniond qend(0.707, 0, 0, 0.707); qend.normalize();

  Eigen::Vector3d omega_start(0.6, 0.5, 0.7);
  Eigen::Vector3d omega_end(2.1, 0.1, 0.1);
  HermiteQuaternionCurve hqc(qstart, omega_start, qend, omega_end);

  // Test that at t=1.0, the quaternion is equal to qend
  Eigen::Quaterniond quat_out;
  std::cout << "hqc(t=1.0) = " << std::endl;
  hqc.evaluate(1.0, quat_out);
  printQuat(quat_out);

  // std::cout << "t," << "qx(t)," << "qy(t)," << "qz(t)" << "qw(t)" << std::endl; 
  // for(int i = 0; i < (N+1); i++){
  //   t = i*dt;
  //   hqc.evaluate(t, quat_out);
  //   std::cout << t << ",";
  //   printQuat(quat_out);
  // }
  std::cout << "target quaternion:" << std::endl;
  printQuat(qend);

  std::cout << "-----end of hermite curves test-------" << std::endl;


}

int main(int argc, char ** argv){
  hermite_curves_test();
  dcm_test();  
  midfeet_test();
  return 0;
}