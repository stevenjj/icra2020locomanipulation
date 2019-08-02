#include <avatar_locomanipulation/planners/locomanipulation_a_star_planner.hpp>

namespace planner{
	// Constructor
	LMVertex::LMVertex(){
		common_initialization();
	}

  LMVertex::LMVertex(double s_in){
    s = s_in;
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

  void LMVertex::setRobotConfig(const Eigen::VectorXd & q_input){
    q_init = q_input;
  }

	// Constructor
	LocomanipulationPlanner::LocomanipulationPlanner(){
    generateDiscretization();    
  }
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

  void LocomanipulationPlanner::generateDiscretization(){
    delta_s_vals = {0.1};
    neighbors.reserve(delta_s_vals.size());
  }

	std::vector< std::shared_ptr<Node> > LocomanipulationPlanner::getNeighbors(shared_ptr<Node> & current){
		current_ = static_pointer_cast<LMVertex>(current);	
    neighbors.clear();

    // Check if this is the first time get Neighbors is being evaluated. 
    bool convergence = false;
    if (first_node_evaluated){
      parent_ = static_pointer_cast<LMVertex>(current_->parent);
      delta_s =  (current_->s - parent_->s);
      convergence = ctg->computeConfigurationTrajectory(f_s, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                                                  parent_->s, delta_s, 
                                                                  parent_->q_init, 
                                                                  input_footstep_list);
      std::cout << "Converged?" << (convergence ? "True" : "False") << std::endl;       
      // If it does not converge, return an empty neighbor list
      if (!convergence){
        return neighbors;
      }
    }else{  
      first_node_evaluated = true;      
    }

    // Generate neighbors with lazy evaluation
    for(int i = 0; i < delta_s_vals.size(); i++){
      // Create the neighbor
      shared_ptr<Node> neighbor (std::make_shared<LMVertex>(current_->s + delta_s_vals[i]));
      // Update the neighbor (probably make this a function to call)
      neighbor_change = static_pointer_cast<LMVertex>(neighbor);
      neighbor_change->parent = current;
      neighbor = static_pointer_cast<Node>(neighbor_change);

      neighbors.push_back(neighbor);

    }


		return neighbors;
	}

	// Print the node path
	void LocomanipulationPlanner::printPath(){
	}

}