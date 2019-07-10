#include <avatar_locomanipulation/ik_module/ik_module.hpp>

// Constructor
IKModule::IKModule(){
  robot_model = std::shared_ptr<ValkyrieModel>(new ValkyrieModel());
}

IKModule::IKModule(std::shared_ptr<ValkyrieModel> robot_model_in){
  robot_model = robot_model_in;
}


// Destructor
IKModule::~IKModule(){}
  

void IKModule::setInitialConfig(const Eigen::VectorXd & q_init){
  q_start = q_init;
}

void IKModule::addTasktoHierarchy(std::shared_ptr<Task> & task_input){
  task_hierarchy.push_back(task_input);
}

void IKModule::clearTaskHierarchy(){
  task_hierarchy.clear();  
}


void IKModule::prepareIKDataStrcutures(){
  // Prepare Data structures for the IK.
  J_.clear();
  N_.clear();
  JN_.clear();
  JNpinv_.clear();
  svd_list_.clear();

  I_ = Eigen::MatrixXd::Identity(robot_model->getDimQdot(), robot_model->getDimQdot());
  Ntmp_ = I_;

  // Construct the Jacobian, Projected Jacobian, SVD, and Projected Nullspace structures:
  for(int i = 0; i < task_hierarchy.size(); i++){
    J_.push_back(Eigen::MatrixXd::Zero(task_hierarchy[i]->task_dim, robot_model->getDimQdot()));   
    JN_.push_back(Eigen::MatrixXd::Zero(task_hierarchy[i]->task_dim, robot_model->getDimQdot()));  
    svd_list_.push_back(Eigen::JacobiSVD<Eigen::MatrixXd>(task_hierarchy[i]->task_dim, robot_model->getDimQdot(), svdOptions) );
    JNpinv_.push_back(Eigen::MatrixXd::Zero(robot_model->getDimQdot(), task_hierarchy[i]->task_dim)); 
  }

  // Construct Null Spaces:
  for(int i = 1; i < task_hierarchy.size(); i++){
    N_.push_back(Eigen::MatrixXd::Identity(robot_model->getDimQdot(), robot_model->getDimQdot()));
    std::cout << "N_" << i-1 << std::endl;
  }

}

void IKModule::updateTaskJacobians(){
  // Gets the task jacobians
  for(int i = 0; i < task_hierarchy.size(); i++){
    task_hierarchy[i]->getTaskJacobian(J_[i]);
  }
}

void IKModule::computePseudoInverses(){
  robot_model->computeInertiaMatrixInverse(q_start);
  updateTaskJacobians();

  // Pseudo inverse and null space of task 1
  // JN_[0] = J_[0];
  // math_utils::weightedPseudoInverse(JN_[0], robot_model->Ainv, svd_list_[0], JNpinv_[0], singular_values_threshold);
  // N_[0] = (I_ - JNpinv_[0]*JN_[0]);      

  // Compute Nullspaces, Projected Task Jacobians, and Pseudoinverse of Projected Jacobians 
  for(int i = 0; i < task_hierarchy.size(); i++){
    // Initialize Nullspace to Identity
    Ntmp_ = I_;
    // Compute N_k = N_1*N_2*...*N_{k-2}*N_{k-1}. Base case N_0 = I
    for(int k = 0; k < i; k++){
      Ntmp_ *= N_[k];
    }
    // Compute Projected Jacobian J_k*N_{k-1}
    if (i == 0){
      JN_[0] = J_[0];
    }else{
      JN_[i] = J_[i]*Ntmp_;      
    }
    // std::cout << "\n i = " << i << std::endl;
    // std::cout << "JN_[i] = " << std::endl;    
    // std::cout << JN_[i] << std::endl;

    // Compute pseudo inverse of JN_[i] 
    math_utils::weightedPseudoInverse(JN_[i], robot_model->Ainv, svd_list_[i], JNpinv_[i], singular_values_threshold);
    // Compute Null Space for this task N_k = I - pinv(J_k N_{k-1})(J_k N_{k-1}).
    if ( i < (task_hierarchy.size()-1) ){
      N_[i] = (I_ - JNpinv_[i]*JN_[i]);      
      // std::cout << "N_[i] = " << std::endl;    
      // std::cout << N_[i] << std::endl;
    }
    // std::cout << "JNpinv_[i] = " << std::endl;    
    // std::cout << JNpinv_[i] << std::endl;
  }

}



bool IKModule::solveIK(int & solve_result, double & error_norm, Eigen::VectorXd & q_sol){
  // Compute inverses
}

