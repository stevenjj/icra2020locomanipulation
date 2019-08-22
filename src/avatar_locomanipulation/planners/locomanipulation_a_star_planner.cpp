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

  LMVertex::LMVertex(double s_in, Eigen::VectorXd & q_init_in){
    s = s_in;
    q_init = q_init_in;
    common_initialization();
  }


	// Destructor
	LMVertex::~LMVertex(){}

	// Common Initialization
	void LMVertex::common_initialization(){
		g_score = 100000;
		f_score = 100000;
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

    // To Do: check that we don't have to run updateFullKinematics before calling getDimQ()
    q_tmp = Eigen::VectorXd::Zero(robot_model->getDimQ());
		f_s = f_s_in;
		ctg = ctg_in;


	}

	// Destructor
	LocomanipulationPlanner::~LocomanipulationPlanner(){}


  bool LocomanipulationPlanner::constructPath(){
    // Clear cached optimal path
    optimal_path.clear();
    // Set Current node to the achieved goal
    shared_ptr<Node> current_node = achieved_goal;
    while (begin->key.compare(current_node->key) != 0) {
      optimal_path.push_back(current_node);
      current_node = current_node->parent;  
    }
    // Add the parent
    optimal_path.push_back(current_node);

    std::cout << "Path has size: " << optimal_path.size() << std::endl;
    std::cout << "Found potential path reconstructing the trajectory..." << std::endl;

    return reconstructConfigurationTrajectory();
  }


  bool LocomanipulationPlanner::reconstructConfigurationTrajectory(){
    // Iterate through optimal path backwards. to construct the forward path
    forward_order_optimal_path.clear();
    for(int i = (optimal_path.size() - 1); i >= 0; i--){
      forward_order_optimal_path.push_back( optimal_path[i] );
    }

    // Reconstruct the configuration trajectory
    Eigen::VectorXd q_begin = Eigen::VectorXd::Zero(robot_model->getDimQ());
    Eigen::VectorXd q_end = Eigen::VectorXd::Zero(robot_model->getDimQ());
    int N_size = ctg->getDiscretizationSize();


    // Initialize the full trajectory size
    double dt_dummy = 1e-3;
    path_traj_q_config.set_dim_N_dt(robot_model->getDimQ(), N_size*(optimal_path.size()-1), dt_dummy);

    int i_run = 0;
    for(int i = 1; i < forward_order_optimal_path.size(); i++){
      std::cout << "reconstructing path. i = " << i << std::endl;
      // Cast Pointers
      current_ = static_pointer_cast<LMVertex>(forward_order_optimal_path[i]);
      parent_ = static_pointer_cast<LMVertex>(current_->parent);

      // Update the input footstep list
      input_footstep_list.clear();
      if (current_->take_a_step){
        input_footstep_list.push_back(current_->footstep);
      }
      // Set initial configuration
      delta_s =  (current_->s - parent_->s);

      bool convergence = false;
      convergence = ctg->computeConfigurationTrajectory(f_s, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                                                  parent_->s, delta_s, 
                                                                  parent_->q_init, 
                                                                 input_footstep_list);
      // Get the final configuration
      std::cout << "getting the final configuration" << std::endl;
      ctg->traj_q_config.get_pos(ctg->getDiscretizationSize() - 1, q_end);

      // Check for convergence. This should work however.
      if (convergence){
        std::cout << "constructing the full path" << std::endl;
        for(int j = 0; j < N_size; j++){
          // Get the configuration at this local trajectory
          ctg->traj_q_config.get_pos(j, q_tmp);
          // Store the trajectory to the global path
          path_traj_q_config.set_pos(i_run + j, q_tmp);        
        }
        // Update i_run
        i_run += N_size;        
      }else{
        return false;
      }

    }

    // To do: update the dt properly for each segment of the trajectory. For now  set a desired dt for visualization
    path_traj_q_config.set_dt(0.05);  // seconds   


    std::cout << "Configuration Path Reconstruction success " << std::endl;
    return true;

  }


	// Locomanipulation gscore
	double LocomanipulationPlanner::gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor){
    current_ = std::static_pointer_cast<LMVertex>(current);
    neighbor_ = std::static_pointer_cast<LMVertex>(neighbor);
		return (neighbor_->s - current_->s);		
	}
	// Locomanipulation heuristic	
	double LocomanipulationPlanner::heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
    neighbor_ = static_pointer_cast<LMVertex>(neighbor);
    goal_ = static_pointer_cast<LMVertex>(goal);
    // Linear distance to the manipulation goal. if w_heuristic = 1.0, we get the true A* result 
    return w_heuristic*(goal_->s - neighbor_->s); 
	}

	// Locomanipulation whether or not the goal was reached
	bool LocomanipulationPlanner::goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal){
    current_ = static_pointer_cast<LMVertex>(current_node);  
    goal_ = static_pointer_cast<LMVertex>(goal);

    // check if s is within 0.05 of goal_s
    std::cout << "testing if we got to the goal for (goal_s, current_s) = (" << goal_->s << ", " << current_->s << ")" << std::endl;
    bool s_satisfaction = (fabs(goal_->s - current_->s) <= goal_tol);
    bool convergence = false;

    // If s is within tolerance, actually check if we can perform this trajectory
    if (s_satisfaction){
      std::cout << "within goal tolerance checking for convergence" << std::endl;
      parent_ = static_pointer_cast<LMVertex>(current_->parent);
      delta_s =  (current_->s - parent_->s);
      // Update the input footstep list
      input_footstep_list.clear();
      if (current_->take_a_step){
        input_footstep_list.push_back(current_->footstep);
      }
      convergence = ctg->computeConfigurationTrajectory(f_s, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                                                  parent_->s, delta_s, 
                                                                  parent_->q_init, 
                                                                 input_footstep_list);

      if (convergence){
        ctg->traj_q_config.get_pos(ctg->getDiscretizationSize() - 1, q_tmp);
        std::cout << "goal q_tmp = " << q_tmp.transpose() << std::endl;
        current_->setRobotConfig(q_tmp);       
      }

    }

    std::cout << "goal reached? " << (s_satisfaction && convergence) << std::endl;
    return (s_satisfaction && convergence);
	}

  void LocomanipulationPlanner::generateDiscretization(){
    delta_s_vals = {0.0, 0.04};

    //number of bins for the local lattice
    int num_x_lattice_pts = int(abs(2*max_lattice_translation)/dx) + 1;
    int num_y_lattice_pts = int(abs(2*max_lattice_translation)/dy) + 1;
    int num_theta_lattice_pts = int(abs(max_lattice_theta - min_lattice_theta)/dtheta);

    // create x coordinate local lattice from -max_lattice_translation to max_lattice_translation spaced by dx
    dx_vals.clear();
    for(int i = 0; i < num_x_lattice_pts; i++){
      dx_vals.push_back( i*dx - max_lattice_translation);
    }
    // create y coordinate local lattice from -max_lattice_translation to max_lattice_translation spaced by dx
    dy_vals.clear();
    for(int i = 0; i < num_y_lattice_pts; i++){
      dy_vals.push_back( i*dy - max_lattice_translation);
    }
    // create dtheta coordinate local lattice from -min_lattice_theta to max_lattice_theta spaced by dtheta
    dtheta_vals.clear();
    for(int i = 0; i < num_y_lattice_pts; i++){
      dy_vals.push_back( i*dtheta - min_lattice_theta);
    }

    neighbors.reserve(delta_s_vals.size());
  }

	std::vector< std::shared_ptr<Node> > LocomanipulationPlanner::getNeighbors(shared_ptr<Node> & current){
    std::cout << "Getting Neighbors" << std::endl;
		current_ = static_pointer_cast<LMVertex>(current);	
    neighbors.clear();

    // Check if this is the first time get Neighbors is being evaluated. 
    bool convergence = false;
    if (first_node_evaluated){
      std::cout << "Testing trajectory feasiblity for the current node" << std::endl;     

      parent_ = static_pointer_cast<LMVertex>(current_->parent);
      delta_s =  (current_->s - parent_->s);
      // Update the input footstep list
      input_footstep_list.clear();
      if (current_->take_a_step){
        input_footstep_list.push_back(current_->footstep);
      }
      convergence = ctg->computeConfigurationTrajectory(f_s, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                                                  parent_->s, delta_s, 
                                                                  parent_->q_init, 
                                                                  input_footstep_list);
      std::cout << "Converged?" << (convergence ? "True" : "False") << std::endl;       

      // If it converges, update the configuration of the current node
      if (convergence){
        // Get the final configuration.
        std::cout << "Current node is feasible. Updating q_tmp" << std::endl;     
        ctg->traj_q_config.get_pos(ctg->getDiscretizationSize() - 1, q_tmp);
        std::cout << "q_tmp = " << q_tmp.transpose() << std::endl;
        current_->setRobotConfig(q_tmp);
      }else{
        // If it does not converge, return an empty neighbor list       
        return neighbors;       
      }

    }else{  
      std::cout << "first node evaluated is true" << std::endl;
      first_node_evaluated = true;      
    }

    std::cout << "generating neighbors for the current neighbor" << std::endl;

    // Generate neighbors with lazy evaluation
    for(int i = 0; i < delta_s_vals.size(); i++){
      // Create the neighbor
      // ensure that s is bounded between 0 and 1.
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