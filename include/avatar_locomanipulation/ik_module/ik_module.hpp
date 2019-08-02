#ifndef ALP_IK_MODULE_H
#define ALP_IK_MODULE_H

#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/tasks/task.hpp>
#include <avatar_locomanipulation/helpers/pseudo_inverse.hpp>
#include <iostream>

#define IK_OPTIMAL_SOL 1	// SUCCESS ||f(x)|| <= error_tol
#define IK_SUBOPTIMAL_SOL 2 // SUCCESS ||grad(f(x))|| <= grad_tol
#define IK_MAX_ITERATIONS_HIT 3 // iter >= MAX_ITERS 
#define IK_MAX_MINOR_ITER_HIT 4  // k_step <= k_step_min

#define IK_VERBOSITY_LOW 0 // No printouts
#define IK_VERBOSITY_HIGH 1 // Printout task errors each iterations

class IKModule{
public:
	IKModule();
	IKModule(std::shared_ptr<RobotModel> & robot_model_in);

	~IKModule();

	std::shared_ptr<RobotModel> robot_model;

	void setRobotModel(std::shared_ptr<RobotModel> & robot_model_in);

	// Sets the initial configuration of the robot
	void setInitialConfig(const Eigen::VectorXd & q_init);

	// Outputs:
	//  return: true if at least the first task in the hierarchy converged
	//			false if the first task in the hierarchy did not converge
	//  params:
	//		solve_result - true if at least the first task converges
	//  	error_norm - the sum of all the task error norms.
	//		q_sol - the configuration vector
	bool solveIK(int & solve_result, double & total_error_norm_out, Eigen::VectorXd & q_sol);
	bool solveIK(int & solve_result, std::vector<double> & task_error_norms, double & total_error_norm_out, Eigen::VectorXd & q_sol);

	// Set verbosity to either IK_VERBOSITY_LOW or IK_VERBOSITY_HIGH.
	// if input <= IK_VERBOSITY_LOW: IK_VERBOSITY_LOW  else if input >= IK_VERBOSITY_HIGH: IK_VERBOSITY_HIGH
	void setVerbosityLevel(int verbosity_level_in);

	// This adds a lower priority task to the hierarchy. 
	void addTasktoHierarchy(std::shared_ptr<Task> & task_input);
	// Clears the task hierarchy
	void clearTaskHierarchy();

	// Call this if this is the first time the IK will be used OR if 
	// the task hierarchy has changed
	void prepareNewIKDataStrcutures();

	// Sets the threshold for the singular values to be set to 0 when performing pseudoinverses with SVD. Default: 1e-4
	void setSingularValueThreshold(double & svd_thresh_in);
	// Sets the maximum iterations to perform descent. Default: 100
	void setMaxIters(int & max_iters_in);
	// Sets the maximum minor iterations (backtracking). Default: 30
	void setMaxMinorIters(int & max_minor_iters_in);

	// Sets the initial descent step. Default: 1.0
	void setDescentStep(int & k_step_in);
	// Sets the backtracking parameter beta with 0.1 <= beta <= 0.8. Default: 0.5 
	void setBackTrackBeta(double & beta_in); 
	// Sets the Task Space Error Norm Tolerance for Optimality Conditions. Default: 1e-4 
	void setErrorTol(double & error_tol_in);
	// Sets the Gradient Norm Tolerance for Sub Optimality. Default: 1e-12
	//	ie: if gradient is too small, we are at a local minimum
	void setGradTol(double & grad_tol_in);
	// Whether or not the inertia matrix is used for a weighted pseudoinverse. Default: false
	void setEnableInertiaWeighting(bool inertia_weighted_in);

	// if true: use dq in the order of task hierarchy and only include next tasks when higher priority tasks have converged
	// if false: uses the total_dq for all the tasks in the hierarchy.
	// Default: true
	void setSequentialDescent(bool sequential_descent_in);
	// if true: uses the current task error norm for computing the backtracking condition. 
	// if false: uses the total error norm for computing the backtracking condition
	// Default: true
	void setBackTrackwithCurrentTaskError(bool backtrack_with_current_task_error_in);
	// if true: during descent, ensures that higher priority tasks are not violated. 
	// if false: assumes that the overall error norm will eventually descend 
	// Default: true
	void setCheckPrevViolations(bool check_prev_violations_in);

	// if true: returns when the first task converges
	// if false : continues to satisfy lower priority tasks
	void setReturnWhenFirstTaskConverges(bool return_when_first_task_converges_in);

	// returns the error tolerance for the problem
	double getErrorTol();

	// Print the latest solution results
	void printSolutionResults();


private:
	// Updates all of the task Jacobians
	void updateTaskJacobians();
	// Compute all the pseudo inverses
	void computePseudoInverses();
	// Compute all the task errors
	void computeTaskErrors();
	// Compute all of dq
	void compute_dq();
	// Compute dq only up to the task being minimized
	void compute_dq(const int & task_idx_to_minimize);

	// Checks if previous tasks have been violated
	bool checkPrevTaskViolation(const int & task_idx_to_minimize);
	// Check if the first task has converged
	bool checkFirstTaskConvergence();

	// Checks the backtracking condition
	bool checkBackTrackCondition(int task_idx_to_minimize);

	// Prints task errors
	void printTaskErrorsHeader();
	void printTaskErrors();

	// Clamps values to the joint limits
	void clampConfig(Eigen::VectorXd & q_config);
	double clampValue(const double & low, double high, const double & value);

	// Containers for configuration and config changes proposal
	Eigen::VectorXd q_start;
	Eigen::VectorXd q_current;
	Eigen::VectorXd q_step;
	Eigen::VectorXd dq_tot;

	// Task hierarchy list
	// A hierarchy of tasks in order of priority
	std::vector< std::shared_ptr<Task> > task_hierarchy;

	Eigen::MatrixXd I_ ; // Identity Matrix
	Eigen::MatrixXd Ntmp_ ; // Temporary Null Space Matrix

	std::vector<Eigen::MatrixXd> J_; // Task Jacobian
	std::vector<Eigen::MatrixXd> N_; // Task Nullspace
	std::vector<Eigen::MatrixXd> JN_; // Projected Task Jacobian
	std::vector<Eigen::MatrixXd> JNpinv_; // Projected Task Jacobian Pseudo Inverse
	std::vector<Eigen::VectorXd> dx_; // Task Errors
	std::vector<double> dx_norms_; // Task Error Norms

	// Sets the svd options and a list of SVDs
	unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;
	std::vector< Eigen::JacobiSVD<Eigen::MatrixXd> > svd_list_; // List of SVD for pseudoinverses

	// IK parameters
	double singular_values_threshold = 1e-4; // Cut off value to treat singular values as 0.0
	int max_iters = 100; // maximum IK iters
	int max_minor_iters  = 30; // maximum number of backtracks
	double k_step = 1.0; // starting step
	double beta = 0.8; // Backtracking Line Search Parameter
	double error_tol = 1e-3; // Task tolerance for success
	double grad_tol = 1e-12;//6; // Gradient descent tolerance for suboptimality
	bool inertia_weighted_ = false;


	// if true: use dq in the order of task hierarchy and only include next tasks when higher priority tasks have converged
	// if false: uses the total_dq for all the tasks in the hierarchy.
	bool sequential_descent = false; 

	// if true: uses the current task error norm for computing the backtracking condition. 
	// if false: uses the total error norm for computing the backtracking condition
	bool backtrack_with_current_task_error = true; 

	// if true: during descent, ensures that higher priority tasks are not violated. 
	// if false: assumes that the overall error norm will eventually descend 
	bool check_prev_violations = false; 

	// if true: the solver immediately returns as soon as the first task converges.
	// if false: the solver tries to satisfy lower priority tasks further
	bool return_when_first_task_converges = false;

	// Errors and Error gradient values:
	double total_error_norm = 0.0;
	double f_q = 0.0;
	double f_q_p_dq = 0.0;
	double grad_f_norm_squared = 0.0;

	// Internal solve result
	bool first_task_convergence = false;
	int solve_result_ = 0;
	Eigen::VectorXd q_sol_;	


	int verbosity_level = IK_VERBOSITY_HIGH;


};

#endif