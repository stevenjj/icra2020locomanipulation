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
		Footstep left_foot;
		Footstep right_foot;
	
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

		virtual std::vector< shared_ptr<Node> > getNeighbors(shared_ptr<Node> & current);

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

	};




}


#endif