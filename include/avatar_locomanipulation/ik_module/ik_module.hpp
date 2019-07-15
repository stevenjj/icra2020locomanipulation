#ifndef ALP_IK_MODULE_H
#define ALP_IK_MODULE_H

#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <avatar_locomanipulation/tasks/task.hpp>
#include <avatar_locomanipulation/helpers/pseudo_inverse.hpp>
#include <iostream>

#define IK_OPTIMAL_SOL 1	// SUCCESS ||f(x)|| <= error_tol
#define IK_SUBOPTIMAL_SOL 2 // SUCCESS ||grad(f(x))|| <= grad_tol
#define IK_MAX_ITERATIONS_HIT 3 // FAILURE iter >= MAX_ITERS 
#define IK_MIN_STEP_HIT 4   // FAILURE k_step <= k_step_min
#define IK_MAX_MINOR_ITER_HIT 5  // FAILURE k_step <= k_step_min


class IKModule{
public:
	IKModule();
	IKModule(std::shared_ptr<ValkyrieModel> robot_model_in);

	~IKModule();

	std::shared_ptr<ValkyrieModel> robot_model;

	// Sets the initial configuration of the robot
	void setInitialConfig(const Eigen::VectorXd & q_init);

	// Outputs:
	//  return: true if optimal or suboptimal solutions were found. 
	//			false if maximum iterations were hit 
	//  params:
	//		solve_result - true if at least the first task converges
	//  	error_norm - the sum of all the task error norms.
	//		q_sol - the configuration vector
	bool solveIK(int & solve_result, double & total_error_norm_out, Eigen::VectorXd & q_sol);

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

	// clamps values to the joint limits
	void clampConfig(Eigen::VectorXd & q_config);

private:
	void updateTaskJacobians();
	void computePseudoInverses();
	void computeTaskErrors();
	void compute_dq();
	void compute_dq(const int & task_idx_to_minimize);


	bool checkPrevTaskViolation(const int & task_idx_to_minimize);
	bool checkFirstTaskConvergence();


	void printTaskErrorsHeader();
	void printTaskErrors();

	double clampValue(const double & low, double high, const double & value);

	unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;

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


	std::vector< Eigen::JacobiSVD<Eigen::MatrixXd> > svd_list_; // List of SVD for pseudoinverses

	// IK parameters
	double singular_values_threshold = 1e-4; // Cut off value to treat singular values as 0.0
	int max_iters = 100; // maximum IK iters
	int max_minor_iters  = 30; // maximum number of backtracks
	double k_step = 1.0; // starting step
	double beta = 0.8; // Backtracking Line Search Parameter
	double error_tol = 1e-4; // Task tolerance for success
	double grad_tol = 1e-12;//6; // Gradient descent tolerance for suboptimality
	bool inertia_weighted_ = false;

	// Errors and Error gradient values:
	double total_error_norm = 0.0;
	double f_q = 0.0;
	double f_q_p_dq = 0.0;
	double grad_f_norm_squared = 0.0;



};

#endif