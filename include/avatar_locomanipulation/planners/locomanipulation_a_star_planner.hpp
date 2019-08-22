#ifndef ALM_LOCOMANIPULATION_A_STAR_PLANNER_H
#define ALM_LOCOMANIPULATION_A_STAR_PLANNER_H

#include <avatar_locomanipulation/planners/a_star_planner.hpp>
#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/data_types/manipulation_function.hpp>
#include <avatar_locomanipulation/data_types/footstep.hpp>

namespace planner{

	class LMVertex: public Node{
	public:
		LMVertex(); // Constructor
		virtual ~LMVertex(); // Destructor

		// Problem specific:
		LMVertex(double s_in, Eigen::VectorXd & q_init_in); // Constructor
		LMVertex(double s_in); // Constructor

		double s = 0.0;
		Eigen::VectorXd q_init;
		bool take_a_step = false;
		Footstep footstep;
	
		void setRobotConfig(const Eigen::VectorXd & q_input);
		void common_initialization();

	};


	class LocomanipulationPlanner: public A_starPlanner{
	public:
		LocomanipulationPlanner();
		virtual ~LocomanipulationPlanner();

		virtual	double gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor);
		virtual double heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal);
		virtual bool goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal);

		virtual bool constructPath();

		virtual std::vector< shared_ptr<Node> > getNeighbors(shared_ptr<Node> & current);

		bool reconstructConfigurationTrajectory();
		void printPath();

		std::shared_ptr<LMVertex> current_;
		std::shared_ptr<LMVertex> goal_;
		std::shared_ptr<LMVertex> neighbor_;		
		std::shared_ptr<LMVertex> opt_node_;

		// Locomanipulation specific member variables and functions.
		void generateDiscretization();

		void initializeLocomanipulationVariables(std::shared_ptr<RobotModel> robot_model_in, std::shared_ptr<ManipulationFunction> f_s_in, std::shared_ptr<ConfigTrajectoryGenerator> ctg_in);
		

		std::shared_ptr<RobotModel> robot_model;
		std::shared_ptr<ManipulationFunction> f_s;
		std::shared_ptr<ConfigTrajectoryGenerator> ctg;
 	  	std::vector<Footstep> input_footstep_list;

 	  	// Discretized values
 	  	std::vector<double> delta_s_vals;
 	  	std::vector<double> dx_vals;
 	  	std::vector<double> dy_vals;
 	  	std::vector<double> dtheta_vals;

 	  	// lattice discretization to be used. with orientation aligned with the starting origin frame but 
 	  	// the translation origin is aligned with the stance frame
 	  	double dx = 0.05; // dx translation discretization
 	  	double dy = 0.05; // dy translation discretization
 	  	double dtheta = 10.0*M_PI/180.0; // 10 degrees of discretization

 	  	double max_lattice_translation = 0.8;
 	  	double max_lattice_theta = M_PI*7.0/8.0;
 	  	double min_lattice_theta = -M_PI*7.0/8.0;

	  	// swing foot kinematic limits w.r.t the stance frame
 	  	double max_reach = 0.4;
 	  	double min_reach = -0.3;

 	  	double max_width = 0.4;
 	  	double min_width = 0.2;

 	  	double max_theta = 0.6; // radians
 	  	double min_theta = -0.15; 






 	  	// Get Neighbors Member Variables 
 	  	double delta_s = 0.0;
 	  	bool first_node_evaluated = false;
		std::vector < std::shared_ptr<Node> > neighbors;
		std::shared_ptr<LMVertex> parent_;
		std::shared_ptr<LMVertex> neighbor_change;

		//goal reached member variables
		double goal_tol = 0.01;
		double w_heuristic = 20;

		// robot_config temp 
 		Eigen::VectorXd q_tmp;


 		// Full path configuration trajectory
		std::vector< shared_ptr<Node> > forward_order_optimal_path;	
		TrajEuclidean   path_traj_q_config; 	// Trajectory of configurations q


	};




}


#endif