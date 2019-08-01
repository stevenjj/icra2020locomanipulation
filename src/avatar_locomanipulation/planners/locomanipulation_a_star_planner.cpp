#include <avatar_locomanipulation/planners/locomanipulation_a_star_planner.hpp>

namespace planner{
	// Constructor
	LMVertex::LMVertex(){
		common_initialization();
	}
	// Destructor
	LMVertex::~LMVertex(){}

	// Common Initialization
	void LMVertex::common_initialization(){
		g_score = 100000;
		f_score = 100000;
		s = 0.0;
		key = (to_string(s) + "-" + to_string(s));	
	}

	// Constructor
	LocomanipulationPlanner::LocomanipulationPlanner(){}
	void LocomanipulationPlanner::initializeLocomanipulationVariables(std::shared_ptr<RobotModel> robot_model_in, std::shared_ptr<ManipulationFunction> f_s_in, std::shared_ptr<ConfigTrajectoryGenerator> ctg_in){
		std::cout << "[LocomanipulationPlanner] Initialized robot model, f_s, and trajectory generation module" << std::endl;
		robot_model = robot_model_in;
		f_s = f_s_in;
		ctg = ctg_in;
	}

	// Destructor
	LocomanipulationPlanner::~LocomanipulationPlanner(){}

	// Locomanipulation gscore
	double LocomanipulationPlanner::gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor){
		return 0.0;		
	}
	// Locomanipulation heuristic	
	double LocomanipulationPlanner::heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
		return 0.0;		
	}
	// Locomanipulation whether or not the goal was reached
	bool LocomanipulationPlanner::goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal){
		return true;
	}

	std::vector< std::shared_ptr<Node> > LocomanipulationPlanner::getNeighbors(shared_ptr<Node> & current){
		current_ = static_pointer_cast<LMVertex>(current);	
    neighbors.clear();

    if (first_pass){
      parent_ = static_pointer_cast<LMVertex>(current_->parent);
      delta_s =  (current_->s - parent_->s);
      bool convergence = ctg->computeConfigurationTrajectory(f_s, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                                                  parent_->s, delta_s, 
                                                                  parent_->q_init, 
                                                                  input_footstep_list);
      std::cout << "Converged?" << (convergence ? "True" : "False") << std::endl;       
    }else{  
      // first_pass = true;      
    }

    // neighbors.push_back()



		return neighbors;
	}

	// Print the node path
	void LocomanipulationPlanner::printPath(){
	}

}