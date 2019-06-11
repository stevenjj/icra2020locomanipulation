#include <avatar_locomanipulation/helpers/hermite_quaternion_curve.hpp>

HermiteQuaternionCurve::HermiteQuaternionCurve(const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
                                               const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end){
  qa = quat_start;
  omega_a = angular_velocity_start;

  qb = quat_end;
  omega_b = angular_velocity_end;

  s_ = 0.0;
  initialize_data_structures();
  std::cout << "[Hermite Quaternion Curve] Initialized" << std::endl;
}

HermiteQuaternionCurve::~HermiteQuaternionCurve(){}

void HermiteQuaternionCurve::initialize_data_structures(){
  q0 = qa;

  if (omega_a.norm() < 1e-6){
    q1 = qa*Eigen::Quaterniond(1, 0, 0, 0);    
  }else{
    q1 = qa*Eigen::Quaterniond( Eigen::AngleAxisd( omega_a.norm()/3.0, omega_a/omega_a.norm() ) ); // q1 = qa*exp(wa/3.0)    
  }

  if (omega_b.norm() < 1e-6){
    q2 = qb*Eigen::Quaterniond(1, 0, 0, 0);    
  }else{  
    q2 = qb*Eigen::Quaterniond( Eigen::AngleAxisd( omega_b.norm()/3.0, omega_b/omega_a.norm() ) ); // q2 = qb*exp(wb/3.0)
  }

  q3 = qb;

  // std::cout << "q0" << std::endl;
  // printQuat(q0);
  // std::cout << "q1" << std::endl;
  // printQuat(q1);
  // std::cout << "q2" << std::endl;
  // printQuat(q2);
  // std::cout << "q3" << std::endl;
  // printQuat(q3);

  // for world frame angular velocities, do: q_1*q_0.inverse(). for local frame do: q_0.inverse()*q_1
  // we will do a global frame specification
  omega_1aa = q1*q0.inverse();
  omega_2aa = q2*q1.inverse();
  omega_3aa = q3*q2.inverse();

  omega_1 = omega_1aa.axis() * omega_1aa.angle(); 
  omega_2 = omega_2aa.axis() * omega_2aa.angle();  
  omega_3 = omega_1aa.axis() * omega_3aa.angle();
}

void HermiteQuaternionCurve::computeBasis(const double & s_in){
  s_ = this->clamp(s_in);
  b1 = 1 - std::pow((1-s_),3);
  b2 = 3*std::pow(s_, 2) - 2*std::pow((s_), 3);
  b3 = std::pow(s_, 3);

  // std::cout << "b1 = " << b1 << std::endl;
  // std::cout << "b2 = " << b2 << std::endl;
  // std::cout << "b3 = " << b3 << std::endl;

  bdot1 = -3*std::pow((1-s_),2);
  bdot2 = 6*s_ - 6*std::pow((s_), 2);
  bdot3 = 3*std::pow((s_), 2);

  bddot1 = -6*(1-s_);
  bddot2 = 6 - 12*s_;
  bddot3 = 6*s_;
}

void HermiteQuaternionCurve::evaluate(const double & s_in, Eigen::Quaterniond & quat_out){
  s_ = this->clamp(s_in);  
  computeBasis(s_);

  qtmp1 = Eigen::AngleAxisd(omega_1aa.angle()*b1, omega_1aa.axis());
  qtmp2 = Eigen::AngleAxisd(omega_2aa.angle()*b2, omega_2aa.axis());
  qtmp3 = Eigen::AngleAxisd(omega_3aa.angle()*b3, omega_3aa.axis());

  // std::cout << "omega_1aa.angle() = " <<  omega_1aa.angle() << std::endl;
  // std::cout << "qtmp1" << std::endl;
  // printQuat(qtmp1);

  // std::cout << "omega_2aa.angle() = " <<  omega_2aa.angle() << std::endl;
  // std::cout << "qtmp2" << std::endl;
  // printQuat(qtmp2);

  // std::cout << "omega_3aa.angle() = " <<  omega_3aa.angle() << std::endl;
  // std::cout << "qtmp3" << std::endl;
  // printQuat(qtmp3);

  quat_out = q0*qtmp1*qtmp2*qtmp3;
}

double HermiteQuaternionCurve::clamp(const double & s_in, double lo, double hi){
    if (s_in < lo){
        return lo;
    }
    else if(s_in > hi){
        return hi;
    }else{
        return s_in;
    }

}

void HermiteQuaternionCurve::printQuat(const Eigen::Quaterniond & quat){
  std::cout <<  quat.x() << " " <<
                quat.y() << " " <<
                quat.z() << " " <<
                quat.w() << " " << std::endl;
}