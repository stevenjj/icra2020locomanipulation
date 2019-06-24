#include <avatar_locomanipulation/data_types/trajectory_orientation.hpp>

TrajOrientation::TrajOrientation(){}

TrajOrientation:: TrajOrientation(const int & N_size_in, const double & dt_in){
  set_N_dt(N_size_in, dt_in);
}

TrajOrientation::~TrajOrientation(){}

double TrajOrientation::get_dt(){
  return dt;
}

int TrajOrientation::get_trajectory_length(){
  return N_size;
}

void TrajOrientation::get_quat(const int & index_in, Eigen::Quaterniond & quat_out){
  quat_out = quat[index_in];

}
void TrajOrientation::get_ang_vel(const int & index_in, Eigen::Vector3d & ang_vel_out){
  ang_vel_out = ang_vel[index_in];  
}

void TrajOrientation::get_ang_acc(const int & index_in, Eigen::Vector3d & ang_acc_out){
  ang_acc_out = ang_acc[index_in];  

}

void TrajOrientation::get_next_quat(Eigen::Quaterniond & quat_out){
  quat_out = quat[index];
  increment_index();
}
void TrajOrientation::get_next_ang_vel(Eigen::Vector3d & ang_vel_out){
  ang_vel_out = ang_vel[index]; 
  increment_index();   
}
void TrajOrientation::get_next_ang_acc(Eigen::Vector3d & ang_acc_out){
  ang_acc_out = ang_acc[index]; 
  increment_index();   
}

void TrajOrientation::get_next_data(Eigen::Quaterniond & quat_out){
  quat_out = quat[index];
  increment_index();
}
void TrajOrientation::get_next_data(Eigen::Quaterniond & quat_out, Eigen::Vector3d & ang_vel_out){
  quat_out = quat[index];
  ang_vel_out = ang_vel[index];
  increment_index();
}
void TrajOrientation::get_next_data(Eigen::Quaterniond & quat_out, Eigen::Vector3d & ang_vel_out, Eigen::Vector3d & ang_acc_out){
  quat_out = quat[index];
  ang_vel_out = ang_vel[index];
  ang_acc_out = ang_acc[index];
  increment_index();
}

void TrajOrientation::increment_index(){
  if (index < (N_size-1) ){
    index++;
  }
}

// Setter functions
void TrajOrientation::set_N_dt(const int & N_size_in, const double & dt_in){
  N_size = N_size_in;
  dt = dt_in;
  index = 0;

  quat.clear(); ang_vel.clear(); ang_acc.clear();
  // Pre-allocate variables:
  for(size_t i = 0; i < N_size; i++){
    quat.push_back(Eigen::Quaterniond(1, 0, 0, 0));
    ang_vel.push_back(Eigen::Vector3d(0,0,0));
    ang_acc.push_back(Eigen::Vector3d(0,0,0));    
  }
}

void TrajOrientation::set_dt(const double & dt_in){
  dt = dt_in;
}

void TrajOrientation::set_quat(const int & index, const Eigen::Quaterniond & quat_in){
  // std::cout << "set quat[" << index << "] = " << quat_in.x() << ", " << quat_in.y() << ", " << quat_in.z() << ", "<< quat_in.w() << std::endl;
  quat[index] = quat_in;    
}

void TrajOrientation::set_ang_vel(const int & index, const Eigen::Vector3d & ang_vel_in){
  ang_vel[index] = ang_vel_in;    

}
void TrajOrientation::set_ang_acc(const int & index, const Eigen::Vector3d & ang_acc_in){
  ang_acc[index] = ang_acc_in;    
}

void TrajOrientation::reset_index(){
  index = 0;
}
