#include <avatar_locomanipulation/data_types/trajectory_euclidean.hpp>

TrajEuclidean::TrajEuclidean(){}

TrajEuclidean:: TrajEuclidean(const int & dim_in, const int & N_size_in, const double & dt_in){
  dim = dim_in;
  N_size = N_size_in;
  dt = dt_in;
  index = 0;
  // Pre-allocate variables:
  for(size_t i = 0; i < N_size; i++){
    pos.push_back(Eigen::VectorXd::Zero(dim));
    vel.push_back(Eigen::VectorXd::Zero(dim));
    acc.push_back(Eigen::VectorXd::Zero(dim));    
  }

}

TrajEuclidean::~TrajEuclidean(){}

double TrajEuclidean::get_dt(){
  return dt;
}
int TrajEuclidean::get_dim(){
  return dim;
}
int TrajEuclidean::get_trajectory_length(){
  return N_size;
}

void TrajEuclidean::get_pos(const int & index_in, Eigen::Ref<Eigen::VectorXd> pos_out){
  pos_out = pos[index_in];
}

void TrajEuclidean::get_vel(const int & index_in, Eigen::Ref<Eigen::VectorXd> vel_out){
  vel_out = vel[index_in];  
}
void TrajEuclidean::get_acc(const int & index_in, Eigen::Ref<Eigen::VectorXd> acc_out){
  acc_out = acc[index_in];  
}

void TrajEuclidean::get_next_pos(Eigen::Ref<Eigen::VectorXd> pos_out){
  pos_out = pos[index];
  index++;
}

void TrajEuclidean::get_next_vel(Eigen::Ref<Eigen::VectorXd> vel_out){
  vel_out = vel[index];
  index++;
}

void TrajEuclidean::get_next_acc(Eigen::Ref<Eigen::VectorXd> acc_out){
  acc_out = acc[index];
  index++;
}

void TrajEuclidean::get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out){
  pos_out = pos[index];
  index++;
}

void TrajEuclidean::get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out, Eigen::Ref<Eigen::VectorXd> vel_out){
  pos_out = pos[index];
  vel_out = vel[index];
  index++;
}

void TrajEuclidean::get_next_data(Eigen::Ref<Eigen::VectorXd> pos_out, Eigen::Ref<Eigen::VectorXd> vel_out, Eigen::Ref<Eigen::VectorXd> acc_out){
  pos_out = pos[index];
  vel_out = vel[index];
  acc_out = acc[index];
  index++;
}

void TrajEuclidean::increment_index(){
  if (index < (N_size-1) ){
    index++;
  }
}

// Setter functions
void TrajEuclidean::set_dim_N_dt(const int & dim_in, const int & N_size_in, const double & dt_in){
  dim = dim_in;
  N_size = N_size_in;
  dt = dt_in;
}
void TrajEuclidean::set_pos(const int & index, const Eigen::VectorXd & pos_in){
  pos[index] = pos_in;  
}
void TrajEuclidean::set_vel(const int & index, const Eigen::VectorXd & vel_in){
  vel[index] = vel_in;
}
void TrajEuclidean::set_acc(const int & index, const Eigen::VectorXd & acc_in){
  acc[index] = acc_in;
}

void TrajEuclidean::reset_index(){
  index = 0;
}