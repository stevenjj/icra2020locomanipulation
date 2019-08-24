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

  LMVertex::LMVertex(double s_in, Eigen::VectorXd & q_init_in, Footstep & left_foot_in, Footstep & right_foot_in){
    s = s_in;
    q_init = q_init_in;
    left_foot.setPosOriSide(left_foot_in.position, left_foot_in.orientation, left_foot_in.robot_side);
    right_foot.setPosOriSide(right_foot_in.position, right_foot_in.orientation, right_foot_in.robot_side);
    common_initialization();
  }

  LMVertex::LMVertex(double s_in, Footstep & left_foot_in, Footstep & right_foot_in){
    s = s_in;
    left_foot = left_foot_in;
    right_foot = right_foot_in;
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


    // Initialize tmp variables
    tmp_pos.setZero();
    tmp_ori.setIdentity();    
	}

	// Destructor
	LocomanipulationPlanner::~LocomanipulationPlanner(){}

  void LocomanipulationPlanner::setStartNode(const shared_ptr<Node> begin_input){
    std::cout << "[LocomanipulationPlanner] Setting the starting node" << std::endl;
    begin = begin_input;
    std::shared_ptr<LMVertex> begin_lmv = static_pointer_cast<LMVertex>(begin);

    // Update the robot model with the starting node initial configuration
    robot_model->updateFullKinematics(begin_lmv->q_init);
    Eigen::Vector3d foot_pos;
    Eigen::Quaterniond foot_ori;

    // Set the right foot position 
    robot_model->getFrameWorldPose("rightCOP_Frame", foot_pos, foot_ori);
    begin_lmv->right_foot.setPosOriSide(foot_pos, foot_ori, RIGHT_FOOTSTEP);
    // begin_lmv->right_foot.printInfo();

    // Set the planner origin to be the starting right foot frame
    planner_origin_pos = foot_pos;
    planner_origin_ori = foot_ori;

    R_planner_origin = planner_origin_ori.toRotationMatrix();
    R_planner_origin_transpose = R_planner_origin.transpose();


    // Set the left position as well
    robot_model->getFrameWorldPose("leftCOP_Frame", foot_pos, foot_ori);
    begin_lmv->left_foot.setPosOriSide(foot_pos, foot_ori, LEFT_FOOTSTEP);
    // begin_lmv->left_foot.printInfo();

    // Compute the midfoot 
    begin_lmv->mid_foot.computeMidfeet(begin_lmv->left_foot, begin_lmv->right_foot, begin_lmv->mid_foot);
    begin_lmv->mid_foot.setMidFoot();
    // begin_lmv->mid_foot.printInfo();

  }

  void LocomanipulationPlanner::setGoalNode(const shared_ptr<Node> goal_input){
    std::cout << "[LocomanipulationPlanner] Setting the goal node" << std::endl;
    goal = goal_input;
    std::shared_ptr<LMVertex> goal_lmv = static_pointer_cast<LMVertex>(goal);

    // Update the robot model with the final node configuration to get the midfeet frame
    robot_model->updateFullKinematics(goal_lmv->q_init);
    Eigen::Vector3d foot_pos;
    Eigen::Quaterniond foot_ori;

    // Set the left and right foot of the node
    robot_model->getFrameWorldPose("rightCOP_Frame", foot_pos, foot_ori);
    goal_lmv->right_foot.setPosOriSide(foot_pos, foot_ori, RIGHT_FOOTSTEP);
    robot_model->getFrameWorldPose("leftCOP_Frame", foot_pos, foot_ori);
    goal_lmv->left_foot.setPosOriSide(foot_pos, foot_ori, LEFT_FOOTSTEP);

    // Compute the midfoot 
    goal_lmv->mid_foot.computeMidfeet(goal_lmv->left_foot, goal_lmv->right_foot, goal_lmv->mid_foot);
    goal_lmv->mid_foot.setMidFoot();
    goal_lmv->mid_foot.printInfo();
  }


  // Converts the input position and orientation 
  // from the world frame to the planner frame
  void LocomanipulationPlanner::convertWorldToPlannerOrigin(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond ori_in,
                                                            Eigen::Vector3d & pos_out, Eigen::Quaterniond & ori_out){
    pos_out = R_planner_origin_transpose*(pos_in - planner_origin_pos);
    ori_out = planner_origin_ori.inverse()*ori_in;
  }
  // Converts the input position and orientation 
  // from the planner frame to the world frame
  void LocomanipulationPlanner::convertPlannerToWorldOrigin(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond ori_in,
                                                            Eigen::Vector3d & pos_out, Eigen::Quaterniond & ori_out){

    pos_out = R_planner_origin*pos_in + planner_origin_pos;
    ori_out = planner_origin_ori*ori_in;
  }


  bool LocomanipulationPlanner::withinKinematicBounds(Footstep & stance_foot, Eigen::Vector3d & landing_pos, Eigen::Quaterniond & landing_ori){
    double sign = (stance_foot.robot_side == RIGHT_FOOTSTEP ? 1.0 : -1.0);     

    foot_frame_landing_foot_pos = stance_foot.R_ori.transpose()*(landing_pos - stance_foot.position);
    foot_frame_landing_foot_ori = stance_foot.orientation.inverse()*landing_ori;

    if ((foot_frame_landing_foot_pos[0] >= min_reach) &&  
        (foot_frame_landing_foot_pos[0] <= max_reach) &&
        (sign*foot_frame_landing_foot_pos[1] >= min_width) &&
        (sign*foot_frame_landing_foot_pos[1] <= max_width) &&
        (acos(foot_frame_landing_foot_ori.w()) >= min_theta) &&
        (acos(foot_frame_landing_foot_ori.w()) <= max_theta)){

        return true;

    }else{
       return false;
    }


  }


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
    delta_s_vals = {0.01, 0.04};

    //number of bins for the local lattice
    int num_x_lattice_pts = int(abs(2*max_lattice_translation)/dx) + 1;
    int num_y_lattice_pts = int(abs(2*max_lattice_translation)/dy) + 1;
    int num_theta_lattice_pts = int(abs(max_lattice_theta - min_lattice_theta)/dtheta) + 1;

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
    for(int i = 0; i < num_theta_lattice_pts; i++){
      dtheta_vals.push_back( i*dtheta - max_lattice_theta);
    }

    // for(int i = 0; i < dx_vals.size(); i++){
    //   std::cout << "x_lattice_pt " << i << " " << dx_vals[i] << std::endl;
    // }
    // for(int i = 0; i < dy_vals.size(); i++){
    //   std::cout << "y_lattice_pt " << i << " " << dy_vals[i] << std::endl;
    // }
    // for(int i = 0; i < dtheta_vals.size(); i++){
    //   std::cout << "dtheta_lattice_pt " << i << " " << dtheta_vals[i] << std::endl;
    // }

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


    // Generate neighbors with lazy evaluation ie: All neighbors are valid unless it has been attempted

    // Generate no step neighbors
    // delta_s
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


    // Initialize footstep landing location object and stance foot location.
    //  These are both in the planner origin frame.
    Footstep landing_foot, stance_foot;
    Eigen::Vector3d   landing_pos, delta_translate; landing_pos.setZero(); delta_translate.setZero();
    Eigen::Quaterniond landing_quat, delta_quat;  landing_quat.setIdentity(); delta_quat.setIdentity();

    // Generate left step neighbors
    // delta_s, dx, dy, dtheta

    // Set stance foot to be the right foot
    // Convert stance foot to planner origin frame
    convertWorldToPlannerOrigin(current_->right_foot.position, current_->right_foot.orientation, tmp_pos, tmp_ori);
    stance_foot.setPosOriSide(tmp_pos, tmp_ori, RIGHT_FOOTSTEP);

    for(int i = 0; i < delta_s_vals.size(); i++){
      for(int j = 0; j < dx_vals.size(); j++){
        for (int k = 0; k < dy_vals.size(); k++){
          for(int m = 0; m < dtheta_vals.size(); m++){
            // Prepare dx, dy, dtheta
            delta_translate[0] = dx_vals[j];
            delta_translate[1] = dy_vals[k];

            delta_quat.x() = 0.0;
            delta_quat.y() = 0.0;
            delta_quat.z() = sin(dtheta_vals[m]);
            delta_quat.w() = cos(dtheta_vals[m]);

            // Compute delta x,y, theta w.r.t the stance
            landing_pos = stance_foot.position + delta_translate;
            landing_quat = stance_foot.orientation*delta_quat;

            // Check if the landing foot is within the kinematic bounds w.r.t. the stance foot
            if (withinKinematicBounds(stance_foot, landing_pos, landing_quat)){
              // std::cout << "accepted" << std::endl;

              // std::cout << "planner frame potential landing_foot_pos = " << landing_pos.transpose() << std::endl;
              // std::cout << "planner frame potential landing_foot_ori = " << acos(landing_quat.w()) << std::endl;             
              // std::cout << "world frame potential landing_foot_pos = " << tmp_pos.transpose() << std::endl;
              // std::cout << "world frame potential landing_foot_ori = " << acos(tmp_ori.w()) << std::endl;             

              // Convert landing location back to the world frame
              convertPlannerToWorldOrigin(landing_pos, landing_quat, tmp_pos, tmp_ori);
              landing_foot.setPosOriSide(tmp_pos, tmp_ori, LEFT_FOOTSTEP);
 
              // Add landing foot if it is within the bounds
              // neighbor.push_back()

            }



          }
        }
      }
    }

    // Generate right step neighbors
    for(int i = 0; i < delta_s_vals.size(); i++){
      for(int j = 0; j < dx_vals.size(); j++){
        for (int k = 0; k < dy_vals.size(); k++){
          for(int m = 0; m < dtheta_vals.size(); m++){
            // prepare dx, dy, dtheta
            // convert proposed left footstep w.r.t right footstep stance
            // Check if proposed theta is within the kinematic bounds
            // Add if it is within the kinematic bounds
          }
        }
      }
    }

		return neighbors;
	}

	// Print the node path
	void LocomanipulationPlanner::printPath(){
	}

}