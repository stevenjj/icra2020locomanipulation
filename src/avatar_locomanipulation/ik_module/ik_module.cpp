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
  dx_.clear();
  dx_norms_.clear();

  q_current = Eigen::VectorXd::Zero(robot_model->getDimQdot());
  q_step = Eigen::VectorXd::Zero(robot_model->getDimQdot());
  dq_tot = Eigen::VectorXd::Zero(robot_model->getDimQdot());
  I_ = Eigen::MatrixXd::Identity(robot_model->getDimQdot(), robot_model->getDimQdot());
  Ntmp_ = I_;

  // Construct the Jacobian, Projected Jacobian, SVD, Projected Nullspace, and task errors structures
  for(int i = 0; i < task_hierarchy.size(); i++){
    J_.push_back(Eigen::MatrixXd::Zero(task_hierarchy[i]->task_dim, robot_model->getDimQdot()));   
    JN_.push_back(Eigen::MatrixXd::Zero(task_hierarchy[i]->task_dim, robot_model->getDimQdot()));  
    JNpinv_.push_back(Eigen::MatrixXd::Zero(robot_model->getDimQdot(), task_hierarchy[i]->task_dim)); 
    svd_list_.push_back(Eigen::JacobiSVD<Eigen::MatrixXd>(task_hierarchy[i]->task_dim, robot_model->getDimQdot(), svdOptions) );
    dx_.push_back(Eigen::VectorXd::Zero(robot_model->getDimQdot()));
    dx_norms_.push_back(0.0);
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

void IKModule::computePseudoInverses(bool inertia_weighted){
  if (inertia_weighted){
    robot_model->computeInertiaMatrixInverse(q_start);
  }
  updateTaskJacobians();

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
    // Compute pseudo inverse of JN_[i] 

    // Inertia Weighted
    if (inertia_weighted){
      math_utils::weightedPseudoInverse(JN_[i], robot_model->Ainv, svd_list_[i], JNpinv_[i], singular_values_threshold);
    }else{
    // Unweighted:
      math_utils::weightedPseudoInverse(JN_[i], I_, svd_list_[i], JNpinv_[i], singular_values_threshold);
    }

    // Compute Null Space for this task N_k = I - pinv(J_k N_{k-1})(J_k N_{k-1}).
    if ( i < (task_hierarchy.size()-1) ){
      N_[i] = (I_ - JNpinv_[i]*JN_[i]);      
    }
  }

}

void IKModule::computeTaskErrors(){
  total_error_norm = 0.0;
  // Compute Task Errors
  for(int i = 0; i < task_hierarchy.size(); i++){
    task_hierarchy[i]->getError(dx_[i]);
    dx_norms_[i] = dx_[i].norm();
    total_error_norm += dx_norms_[i];
  }
  printTaskErrors();
}

void IKModule::printTaskErrorsHeader(){
  std::cout << "Errors: ";
  for(int i = 0; i < dx_.size(); i++){
      std::cout << "dx[" << i << "], " ;
  } 
  std::cout << "||total_error_norm||" << std::endl;
}

void IKModule::printTaskErrors(){
  for(int i = 0; i < dx_.size(); i++){
      std::cout << dx_norms_[i] << ", ";
  }   
  std::cout << total_error_norm << std::endl;
}


void IKModule::compute_dq(){
  computePseudoInverses();
  computeTaskErrors();
  // Compute dq_tot
  for(int i = 0; i < task_hierarchy.size(); i++){
    if (i == 0){
      dq_tot = JNpinv_[0]*dx_[0];
    }else{
      dq_tot = dq_tot + (JNpinv_[i]*(dx_[i] - J_[i]*dq_tot));
    }
  }

}


bool IKModule::solveIK(int & solve_result, double & error_norm, Eigen::VectorXd & q_sol, bool inertia_weighted){
  q_current = q_start;

  for(int i = 0; i < max_iters; i++){
    if (i == 0){
      printTaskErrorsHeader();
    }
    compute_dq();

    robot_model->forwardIntegrate(q_current, 1.0*dq_tot, q_step);
    q_current = q_step;
    robot_model->updateFullKinematics(q_step);
  }

  // Return Max Iters 
  solve_result = IK_MAX_ITERATIONS_HIT; 
  error_norm = total_error_norm;
  q_sol = q_current;
}

