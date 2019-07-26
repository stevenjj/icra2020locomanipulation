#include <avatar_locomanipulation/ik_module/ik_module.hpp>

// Constructor
IKModule::IKModule(){
}

IKModule::IKModule(std::shared_ptr<RobotModel> & robot_model_in){
  setRobotModel(robot_model_in);
}

// Destructor
IKModule::~IKModule(){}
  
void IKModule::setRobotModel(std::shared_ptr<RobotModel> & robot_model_in){
  robot_model = robot_model_in; 
}

void IKModule::setInitialConfig(const Eigen::VectorXd & q_init){
  q_start = q_init;
}

void IKModule::addTasktoHierarchy(std::shared_ptr<Task> & task_input){
  task_hierarchy.push_back(task_input);
}

void IKModule::clearTaskHierarchy(){
  task_hierarchy.clear();  
}


void IKModule::prepareNewIKDataStrcutures(){
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
  q_sol_ = Eigen::VectorXd::Zero(robot_model->getDimQdot());
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
    // std::cout << "N_" << i-1 << std::endl;
  }

}


// Sets the threshold for the SVD when performing pseudoinverses. Default: 1e-4
void IKModule::setSingularValueThreshold(double & svd_thresh_in){
  singular_values_threshold = svd_thresh_in;
}
// Sets the maximum iterations to perform descent. Default: 100
void IKModule::setMaxIters(int & max_iters_in){
  max_iters = max_iters_in;
}

// Sets the maximum minor iterations (backtracking). Default: 30
void IKModule::setMaxMinorIters(int & max_minor_iters_in){
  max_minor_iters = max_minor_iters_in;
}


// Sets the initial descent step. Default: 1.0
void IKModule::setDescentStep(int & k_step_in){
  k_step = k_step_in;
}
// Sets the backtracking parameter beta with 0.1 <= beta <= 0.8. Default: 0.5 
void IKModule::setBackTrackBeta(double & beta_in){
  beta = beta_in;
}
// Sets the task Space Error Norm Tolerance for Optimality Conditions. Default: 1e-4 
void IKModule::setErrorTol(double & error_tol_in){
  error_tol = error_tol_in;
}
// Sets the Gradient Norm Tolerance for Sub Optimality. Default: 1e-12
//  ie: if gradient is too small, we are at a local minimum
void IKModule::setGradTol(double & grad_tol_in){
  grad_tol = grad_tol_in;
}
// Whether or not the inertia matrix is used for a weighted pseudoinverse. Default: false
void IKModule::setEnableInertiaWeighting(bool inertia_weighted_in){
  inertia_weighted_ = inertia_weighted_in;
}

// if true: use dq in the order of task hierarchy and only include next tasks when higher priority tasks have converged
// if false: uses the total_dq for all the tasks in the hierarchy.
void IKModule::setSequentialDescent(bool sequential_descent_in){
  sequential_descent = sequential_descent_in;
}
// if true: uses the current task error norm for computing the backtracking condition. 
// if false: uses the total error norm for computing the backtracking condition
void IKModule::setBackTrackwithCurrentTaskError(bool backtrack_with_current_task_error_in){
  backtrack_with_current_task_error = backtrack_with_current_task_error_in;
}
// if true: during descent, ensures that higher priority tasks are not violated. 
// if false: assumes that the overall error norm will eventually descend 
void IKModule::setCheckPrevViolations(bool check_prev_violations_in){
  check_prev_violations = check_prev_violations_in;
}

void IKModule::setReturnWhenFirstTaskConverges(bool return_when_first_task_converges_in){
  return_when_first_task_converges = return_when_first_task_converges_in;
}

void IKModule::setVerbosityLevel(int verbosity_level_in){
  if (verbosity_level_in <= IK_VERBOSITY_LOW){
    verbosity_level = IK_VERBOSITY_LOW;
  }else if (verbosity_level_in >= IK_VERBOSITY_HIGH){
    verbosity_level = IK_VERBOSITY_HIGH;    
  }
}


double IKModule::getErrorTol(){
  return error_tol;
}

void IKModule::updateTaskJacobians(){
  // Gets the task jacobians
  for(int i = 0; i < task_hierarchy.size(); i++){
    task_hierarchy[i]->getTaskJacobian(J_[i]);
  }
}

void IKModule::computePseudoInverses(){
  if (inertia_weighted_){
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
    if (inertia_weighted_){
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
  if (verbosity_level == IK_VERBOSITY_HIGH){
    printTaskErrors();
  }
}

void IKModule::printTaskErrorsHeader(){
  std::cout << "Iteration type, ||total_error_norm||, " ;
  for(int i = 0; i < dx_.size(); i++){
      std::cout << "dx[" << i << "], " ;
  }
  std::cout << std::endl; 
}

void IKModule::printTaskErrors(){
  std::cout << total_error_norm << "| ";
  for(int i = 0; i < dx_.size(); i++){
      std::cout << dx_norms_[i] << ", ";
  }
  std::cout << std::endl;
}

void IKModule::compute_dq(const int & task_idx_to_minimize){
  // Compute dq_tot
  for(int i = 0; i < task_hierarchy.size(); i++){
    // Break after we have optimized current task.
    if (i > task_idx_to_minimize){
      break;
    }else{
      if (i == 0){
        dq_tot = JNpinv_[0]*dx_[0];
      }else{
        dq_tot = dq_tot + (JNpinv_[i]*(dx_[i] - J_[i]*dq_tot));
      }      
    }

  }  
}

void IKModule::compute_dq(){
  // Compute dq_tot for all tasks
  for(int i = 0; i < task_hierarchy.size(); i++){
    if (i == 0){
      dq_tot = JNpinv_[0]*dx_[0];
    }else{
      dq_tot = dq_tot + (JNpinv_[i]*(dx_[i] - J_[i]*dq_tot));
    }
  }

}

bool IKModule::checkPrevTaskViolation(const int & task_idx_to_minimize){
  // Check if the new configuration significantly altered previous tasks:
  for(int j = 0; j < (task_idx_to_minimize); j++){
    if (dx_norms_[j] > error_tol){
      // std::cout << "previous task violated" << std::endl;
      return true;
    }    
  }
  return false;
}

// Checks if the first task has converged
bool IKModule::checkFirstTaskConvergence(){
  if (dx_norms_[0] < error_tol){
    // std::cout << "Task 1 converged within error tolerance." << std::endl;
    first_task_convergence = true;
  }else{
    // std::cout << "Task 1 did not converge within error tolerance." << std::endl;
    first_task_convergence = false;
  }
  return first_task_convergence;
}

bool IKModule::checkBackTrackCondition(int task_idx_to_minimize){
  // At each descent step, whether or not to ensure that the previous task was not violated
  if (check_prev_violations){
      //    - the new error is larger than the previous error.
      //    - or if the new configuration significantly altered previous tasks:
    return ((f_q_p_dq > f_q) || checkPrevTaskViolation(task_idx_to_minimize));
  }else{
    // Assume that we can descend anyway
    //    - the new error is larger than the previous error.
    return (f_q_p_dq > f_q);
  }
}

bool IKModule::solveIK(int & solve_result, std::vector<double> & task_error_norms, double & total_error_norm_out, Eigen::VectorXd & q_sol){
  bool convergence = solveIK(solve_result, total_error_norm_out, q_sol);
  task_error_norms = dx_norms_;
  return convergence;
}


bool IKModule::solveIK(int & solve_result, double & total_error_norm_out, Eigen::VectorXd & q_sol){
  q_current = q_start;

  int task_idx_to_minimize = 0;

  for(int i = 0; i < max_iters; i++){
    // First pass
    if (i == 0){
      robot_model->updateFullKinematics(q_current);
      if (verbosity_level == IK_VERBOSITY_HIGH){
        printTaskErrorsHeader();
        std::cout << "Major Iter " << i << ": ";        
      }
      computeTaskErrors();
    }else{
      if (verbosity_level == IK_VERBOSITY_HIGH){
        std::cout << "Major Iter " << i << ": " << std::endl;      
      } 
    }


    // Check task convergence in order of priority
    f_q = 0;
    if (backtrack_with_current_task_error){
      // Using the current task error norm as the condition for back tracking
      for(int j = 0; j < dx_norms_.size(); j++){
        if (dx_norms_[j] < error_tol){
          continue; 
        }else{
          f_q = dx_norms_[j];
          task_idx_to_minimize = j;
          break;
        }
      }     
    }else{
      // Using the total error norm as the condition for back tracking
      f_q = total_error_norm;
    }


    // Compute PseudoInverses and Find Descent Direction
    computePseudoInverses();
    if (sequential_descent){
      // Compute dq of lower priorty tasks if higher priority tasks have converged already
      compute_dq(task_idx_to_minimize);
    }else{
      // Compute dq for all tasks
      compute_dq();
    }

    // Start Gradient Descent with backtracking
    k_step = 1.0;
    int minor_iter_count = 0;
    while(true){
      // Forward integrate with the computed dq
      robot_model->forwardIntegrate(q_current, k_step*dq_tot, q_step);
      // Ensure joint limits are not exceeded
      clampConfig(q_step);
      // Update the robot model
      robot_model->updateFullKinematics(q_step);

      if (verbosity_level == IK_VERBOSITY_HIGH){
        std::cout << "  Minor Iter " << minor_iter_count << ": ";        
      } 
      // Compute errors for this configuration change proposal
      computeTaskErrors();

      // Check if the first task has converged and if we are required to return immediately. 
      if (return_when_first_task_converges && checkFirstTaskConvergence()){
        solve_result = IK_OPTIMAL_SOL;
        solve_result_ = IK_OPTIMAL_SOL;
        total_error_norm_out = total_error_norm;
        q_sol = q_current;
        q_sol_ = q_current;
        return checkFirstTaskConvergence();
      }

      if (backtrack_with_current_task_error){
        // Using the current task error norm as the condition for back tracking
        f_q_p_dq = dx_norms_[task_idx_to_minimize]; // compare with current task
      }else{
        // Using the total error norm as the condition for back tracking
        f_q_p_dq = total_error_norm; // compare with total error
      }

      // Check if we hit minor iterations limit
      minor_iter_count++;
      if (minor_iter_count > max_minor_iters){
        // std::cout << "[IK Module] Max Minor Iters Hit " << std::endl;        
        solve_result = IK_MAX_MINOR_ITER_HIT;
        solve_result_ = IK_MAX_MINOR_ITER_HIT;
        total_error_norm_out = total_error_norm;
        q_sol = q_current;
        q_sol_ = q_current;

        // Recompute errors for the solution
        robot_model->updateFullKinematics(q_sol);
        computeTaskErrors();
        return checkFirstTaskConvergence(); // true if at least the first task converged
      }

      // Check if the gradient is too small. If so, we are at a local minimum
      grad_f_norm_squared = k_step*std::pow(dq_tot.norm(), 2);
      if (grad_f_norm_squared < grad_tol){
        // std::cout << "[IK Module] Sub Optimal Solution" << std::endl;
        solve_result = IK_SUBOPTIMAL_SOL;
        solve_result_ = IK_SUBOPTIMAL_SOL;
        total_error_norm_out = total_error_norm;
        q_sol = q_current;
        q_sol_ = q_current;
        // Recompute errors for the solution
        robot_model->updateFullKinematics(q_sol);
        computeTaskErrors();
        return checkFirstTaskConvergence();  // true if at least the first task converged
      }
      

      // Backtrack
      if (checkBackTrackCondition(task_idx_to_minimize)) {
      // if (f_q_p_dq > f_q) {
        // Decrease the step size
        k_step = beta*k_step;        
        // std::cout << "backtracking with k_step = " << k_step << std::endl;
        continue; // Backtrack and retry descent
      }else{
        // Successfully found a descent vector
        q_current = q_step;
        break; // finish back tracking step
      }
    }

    // Check if we reached the optimality condition
    if (total_error_norm < error_tol){
      // std::cout << "[IK Module] Optimal Solution" << std::endl;
      solve_result = IK_OPTIMAL_SOL;
      solve_result_ = IK_OPTIMAL_SOL;
      total_error_norm_out = total_error_norm;
      q_sol = q_current;
      q_sol_ = q_current;
      return checkFirstTaskConvergence();
    }


  }

  // Hit maximum iterations
  std::cout << "[IK Module] Maximum Iterations Hit" << std::endl;  
  solve_result = IK_MAX_ITERATIONS_HIT; 
  solve_result_ = IK_MAX_ITERATIONS_HIT;
  total_error_norm_out = total_error_norm;
  q_sol = q_current;
  q_sol_ = q_current;
  return false;
}

void IKModule::clampConfig(Eigen::VectorXd & q_config){
  for(int i = 0; i < q_config.size(); i++){
    q_config[i] = clampValue(robot_model->q_lower_pos_limit[i], robot_model->q_upper_pos_limit[i], q_config[i]);
  }
}


double IKModule::clampValue(const double & low, double high, const double & value){
    if (value < low){
        return low;
    }
    else if(value > high){
        return high;
    }else{
        return value;
    }

}

void IKModule::printSolutionResults(){
  std::cout << std::endl;
  std::cout << "[IK Module] First Task Convergence: " << first_task_convergence << std::endl;
  if (solve_result_ == IK_OPTIMAL_SOL){
    std::cout << "[IK Module] Result: Optimal Solution" << std::endl;
  } else if (solve_result_ == IK_SUBOPTIMAL_SOL){
    std::cout << "[IK Module] Result: Sub-Optimal Solution" << std::endl;    
  } else if (solve_result_ == IK_MAX_ITERATIONS_HIT){
    std::cout << "[IK Module] Result: Maximum Iterations Hit" << std::endl;    
  } else if (solve_result_ == IK_MAX_MINOR_ITER_HIT){
    std::cout << "[IK Module] Result: Maximum Minor Iterations Hit" << std::endl;    
  } else{
    std::cout << "[IK Module] Result: No solve routine has been called yet" << std::endl;        
  }

  std::cout << "[IK Module] Task error norms:" << std::endl;
  std::cout << "    ";
  for(int i = 0; i < dx_.size(); i++){
      std::cout << "dx[" << i << "] " ;
  }
  std::cout << std::endl;
  std::cout << "    ";
  for(int i = 0; i < dx_.size(); i++){
      std::cout << dx_norms_[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "[IK Module] Total error norm = " << total_error_norm << std::endl;

  std::cout << "[IK Module] configuration vector q_sol = " ;
  for(int i = 0; i < q_sol_.size(); i++){
    std::cout << q_sol_[i] << ", ";
  }   
  std::cout << std::endl << std::endl;

}