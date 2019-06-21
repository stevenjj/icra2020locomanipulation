#include <avatar_locomanipulation/data_types/trajectory_SE3.hpp>

TrajSE3::TrajSE3(){}

TrajSE3:: TrajSE3(const int & N_size_in, const double & dt_in){
  set_N_dt(N_size_in, dt_in);
}

TrajSE3::~TrajSE3(){}

double TrajSE3::get_dt(){
  return dt;
}

int TrajSE3::get_trajectory_length(){
  return N_size;
}

void TrajSE3::reset_index(){
  traj_pos.reset_index();
  traj_ori.reset_index();
}

// Setter functions
void TrajSE3::set_N_dt(const int & N_size_in, const double & dt_in){
  traj_pos = TrajEuclidean(3, N_size_in, dt_in);
  traj_ori = TrajOrientation(N_size_in, dt_in);
}


// All of the get_next functions increment the internal index counter
void TrajSE3::get_pos(const int & index_in, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out){
  traj_pos.get_pos(index_in, pos_out);
  traj_ori.get_quat(index_in, quat_out);
}
void TrajSE3::get_vel(const int & index_in, Eigen::Vector3d & vel_out, Eigen::Vector3d & ang_vel_out){
  traj_pos.get_vel(index_in, vel_out);
  traj_ori.get_ang_vel(index_in, ang_vel_out);  
}
void TrajSE3::get_acc(const int & index_in, Eigen::Vector3d & acc_out, Eigen::Vector3d & ang_acc_out){
  traj_pos.get_acc(index_in, acc_out);
  traj_ori.get_ang_acc(index_in, ang_acc_out);    
}

void TrajSE3::get_next_pos(Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out){
  traj_pos.get_next_pos(pos_out);
  traj_ori.get_next_quat(quat_out);
}

void TrajSE3::get_next_vel(Eigen::Vector3d & vel_out, Eigen::Vector3d & ang_vel_out){
  traj_pos.get_next_vel(vel_out);
  traj_ori.get_next_ang_vel(ang_vel_out);
}

void TrajSE3::get_next_acc(Eigen::Vector3d & acc_out, Eigen::Vector3d & ang_acc_out){
  traj_pos.get_next_acc(acc_out);
  traj_ori.get_next_ang_acc(ang_acc_out);
}

void TrajSE3::get_next_data(Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out){
  traj_pos.get_next_data(pos_out);
  traj_ori.get_next_data(quat_out);  
}

void TrajSE3::get_next_data(Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out,
                            Eigen::Vector3d & vel_out, Eigen::Vector3d & ang_vel_out){
  traj_pos.get_next_data(pos_out, vel_out);
  traj_ori.get_next_data(quat_out, ang_vel_out);
}

void TrajSE3::get_next_data(Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out,
             Eigen::Vector3d & vel_out, Eigen::Vector3d & ang_vel_out,
             Eigen::Vector3d & acc_out, Eigen::Vector3d & ang_acc_out){
  traj_pos.get_next_data(pos_out, vel_out, acc_out);
  traj_ori.get_next_data(quat_out, ang_vel_out, ang_acc_out);  
}

void TrajSE3::set_pos(const int & index, const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in){
  traj_pos.set_pos(index, pos_in);
  traj_ori.set_quat(index, quat_in);
}

void TrajSE3::set_vel(const int & index, const Eigen::Vector3d & vel_in, const Eigen::Vector3d & ang_vel_in){
  traj_pos.set_vel(index, vel_in);
  traj_ori.set_ang_vel(index, ang_vel_in);
}

void TrajSE3::set_acc(const int & index, const Eigen::Vector3d & acc_in, const Eigen::Vector3d & ang_acc_in){
  traj_pos.set_acc(index, acc_in);
  traj_ori.set_ang_acc(index, ang_acc_in);
}
