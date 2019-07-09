#include <avatar_locomanipulation/ik_module/ik_module.hpp>

// Constructor
IKModule::IKModule(){
  robot_model = std::shared_ptr<ValkyrieModel>(new ValkyrieModel());
}

// Destructor
IKModule::~IKModule(){}


void IKModule::setInitialConfig(const Eigen::VectorXd & q_init){
  q_start = q_init;
}

bool IKModule::solveIK(int & solve_result, double & error_norm, Eigen::VectorXd & q_sol){
}
