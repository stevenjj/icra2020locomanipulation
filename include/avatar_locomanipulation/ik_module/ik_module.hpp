#ifndef IK_MODULE_H
#define IK_MODULE_H

#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <avatar_locomanipulation/tasks/task.hpp>
#include <iostream>

#define IK_OPTIMAL_SOL 1	// SUCCESS ||f(x)|| <= error_tol
#define IK_SUBOPTIMAL_SOL 2 // SUCCESS ||grad(f(x))|| <= grad_tol
#define IK_MAX_ITERATIONS_HIT 3 // FAILURE iter >= MAX_ITERS 
#define IK_MIN_STEP_HIT 4   // FAILURE k_step <= k_step_min


class IKModule{
public:
	IKModule();
	~IKModule();

	std::shared_ptr<ValkyrieModel> robot_model;

	void setInitialConfig(const Eigen::VectorXd & q_init);

	bool solveIK(int & solve_result, double & error_norm, Eigen::VectorXd & q_sol);


 // 
	// void addTasktoHierarchy;
	// std::vector<>

private:
	Eigen::VectorXd q_start;
	Eigen::VectorXd q_current;

	// Task hierarchy list
	std::vector<Task> task_hierarchy;
};

#endif