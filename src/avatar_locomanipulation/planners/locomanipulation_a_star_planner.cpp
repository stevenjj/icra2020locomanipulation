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
    // Compute the midfeet based on the left and right feet
    mid_foot.computeMidfeet(left_foot, right_foot, mid_foot);
    mid_foot.setMidFoot();
    common_initialization();
  }

  LMVertex::LMVertex(double s_in, Footstep & left_foot_in, Footstep & right_foot_in){
    s = s_in;
    left_foot.setPosOriSide(left_foot_in.position, left_foot_in.orientation, left_foot_in.robot_side);
    right_foot.setPosOriSide(right_foot_in.position, right_foot_in.orientation, right_foot_in.robot_side);
    // Compute the midfeet based on the left and right feet
    mid_foot.computeMidfeet(left_foot, right_foot, mid_foot);
    mid_foot.setMidFoot();
    common_initialization();
  }


  LMVertex::LMVertex(double s_in, Eigen::VectorXd & q_init_in){
    s = s_in;
    q_init = q_init_in;
    common_initialization();
  }


  // Destructor
  LMVertex::~LMVertex(){}

  double LMVertex::getAngle(const Eigen::Quaterniond & quat_in){
    tmp_aa = quat_in;
    // Return z axis of omega_hat*angle
    return (tmp_aa.axis()*tmp_aa.angle())[2];
  }

  // Common Initialization
  void LMVertex::common_initialization(){
    g_score = 100000;
    f_score = 100000;
    double factor = 1000.0;
    key = (to_string((int)round(s*factor)) + "/" + "lx:" + to_string((int)round(left_foot.position[0] * factor)) + "-"
                                                 + "ly:" + to_string((int)round(left_foot.position[1] * factor)) + "-"
                                                 + "lt:" + to_string((int)round(getAngle(left_foot.orientation) * factor)) + "/"
                                                 + "rx:" + to_string((int)round(right_foot.position[0] * factor)) + "-"
                                                 + "ry:" + to_string((int)round(right_foot.position[1] * factor)) + "-"
                                                 + "rt:" + to_string((int)round(getAngle(right_foot.orientation) * factor))
          );


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

    // Initialize the config trajectory discretization
    ctg->initializeDiscretization(N_size_per_edge);    

    // Init feet position w.r.t hand 
    lf_pos_wrt_hand.setZero();
    rf_pos_wrt_hand.setZero();

    lf_ori_wrt_hand.setIdentity();
    rf_ori_wrt_hand.setIdentity();

    lf_guess_pos.setZero();
    lf_guess_ori.setIdentity();

    rf_guess_pos.setZero();
    rf_guess_ori.setIdentity();

    // Initialize tmp variables
    tmp_pos.setZero();
    tmp_ori.setIdentity();    

    nn_stance_origin = CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE; 
    nn_manipulation_type = CONTACT_TRANSITION_DATA_RIGHT_HAND; 

    feasibility_stance_foot_pos.setZero();
    feasibility_stance_foot_ori.setIdentity();

    nn_swing_foot_start_pos.setZero();
    nn_swing_foot_start_ori.setIdentity();

    nn_pelvis_pos.setZero();
    nn_pelvis_ori.setIdentity();

    nn_landing_foot_pos.setZero();
    nn_landing_foot_ori.setIdentity();

    nn_right_hand_start_pos.setZero();
    nn_right_hand_start_ori.setIdentity();

    nn_left_hand_start_pos.setZero();
    nn_left_hand_start_ori.setIdentity();

    tmp_ori_vec3.setZero();

  }

  // Destructor
  LocomanipulationPlanner::~LocomanipulationPlanner(){}

  void LocomanipulationPlanner::setStartNode(const shared_ptr<Node> begin_input){
    std::cout << "[LocomanipulationPlanner] Setting the starting node" << std::endl;
    begin = begin_input;
    begin_ = static_pointer_cast<LMVertex>(begin);

    // Set flag that this is a start node
    begin_->isStartNode = true;

    // Update the robot model with the starting node initial configuration
    robot_model->updateFullKinematics(begin_->q_init);
    // Set the starting pelvis height
    nn_starting_pelvis_height = begin_->q_init[2]; // Get the z height of the pelvis

    Eigen::Vector3d foot_pos;
    Eigen::Quaterniond foot_ori;

    // Set the right foot position 
    robot_model->getFrameWorldPose("rightCOP_Frame", foot_pos, foot_ori);
    begin_->right_foot.setPosOriSide(foot_pos, foot_ori, RIGHT_FOOTSTEP);
    // begin_->right_foot.printInfo();

    // Set the planner origin to be the starting right foot frame
    planner_origin_pos = foot_pos;
    planner_origin_ori = foot_ori;

    R_planner_origin = planner_origin_ori.toRotationMatrix();
    R_planner_origin_transpose = R_planner_origin.transpose();

    // Set the left position as well
    robot_model->getFrameWorldPose("leftCOP_Frame", foot_pos, foot_ori);
    begin_->left_foot.setPosOriSide(foot_pos, foot_ori, LEFT_FOOTSTEP);
    // begin_->left_foot.printInfo();

    // Compute the midfoot 
    begin_->mid_foot.computeMidfeet(begin_->left_foot, begin_->right_foot, begin_->mid_foot);
    begin_->mid_foot.setMidFoot();
    // begin_->mid_foot.printInfo();

    // Set Feet position w.r.t hand 
    std::cout << "normal start node finished" << std::endl;
    double s_init = 0.0;
    f_s->getPose(s_init, tmp_pos, tmp_ori);
    Eigen::Matrix3d R_hand_ori_T = tmp_ori.toRotationMatrix().transpose();

    lf_pos_wrt_hand = R_hand_ori_T*(begin_->left_foot.position - tmp_pos);
    rf_pos_wrt_hand = R_hand_ori_T*(begin_->right_foot.position - tmp_pos);

    lf_ori_wrt_hand = tmp_ori.inverse()*begin_->left_foot.orientation;
    rf_ori_wrt_hand = tmp_ori.inverse()*begin_->right_foot.orientation;
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

    // Get a copy of the goal node s variable
    goal_s = goal_lmv->s;

  }

  double LocomanipulationPlanner::getAngle(const Eigen::Quaterniond & quat_in){
    tmp_aa = quat_in;
    return (tmp_aa.axis()*tmp_aa.angle())[2];
  }


  // Converts the input position and orientation 
  // from the world frame to the planner frame
  void LocomanipulationPlanner::convertWorldToPlannerOrigin(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & ori_in,
                                                            Eigen::Vector3d & pos_out, Eigen::Quaterniond & ori_out){
    pos_out = R_planner_origin_transpose*(pos_in - planner_origin_pos);
    ori_out = planner_origin_ori.inverse()*ori_in;
  }
  // Converts the input position and orientation 
  // from the planner frame to the world frame
  void LocomanipulationPlanner::convertPlannerToWorldOrigin(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & ori_in,
                                                            Eigen::Vector3d & pos_out, Eigen::Quaterniond & ori_out){

    pos_out = R_planner_origin*pos_in + planner_origin_pos;
    ori_out = planner_origin_ori*ori_in;
  }


  // Convert world to stance frame defined by feasibility_stance_foot_pos and feasibility_stance_foot_ori
  void LocomanipulationPlanner::convertWorldToStanceOrigin(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & ori_in,
                                                            Eigen::Vector3d & pos_out, Eigen::Quaterniond & ori_out){
    pos_out = R_stance_origin_transpose*(pos_in - feasibility_stance_foot_pos);
    ori_out = feasibility_stance_foot_ori.inverse()*ori_in;
  }


  bool LocomanipulationPlanner::withinKinematicBounds(Footstep & stance_foot, Eigen::Vector3d & landing_pos, Eigen::Quaterniond & landing_ori){
    double sign = (stance_foot.robot_side == RIGHT_FOOTSTEP ? 1.0 : -1.0);     

    foot_frame_landing_foot_pos = stance_foot.R_ori.transpose()*(landing_pos - stance_foot.position);
    foot_frame_landing_foot_ori = stance_foot.orientation.inverse()*landing_ori;

    if ((foot_frame_landing_foot_pos[0] >= min_reach) &&  
        (foot_frame_landing_foot_pos[0] <= max_reach) &&
        (sign*foot_frame_landing_foot_pos[1] >= min_width) &&
        (sign*foot_frame_landing_foot_pos[1] <= max_width) &&
        (getAngle(foot_frame_landing_foot_ori) >= min_theta) &&
        (getAngle(foot_frame_landing_foot_ori) <= max_theta)){

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
    std::cout << "Found potential path with size " << optimal_path.size() << ". Reconstructing the trajectory..." << std::endl;

    return reconstructConfigurationTrajectory();
    // return reconstructConfigurationTrajectoryv2();
  }

  bool LocomanipulationPlanner::reconstructConfigurationTrajectoryv2(){
    // Iterate through optimal path backwards. to construct the forward path
    forward_order_optimal_path.clear();
    for(int i = (optimal_path.size() - 1); i >= 0; i--){
      forward_order_optimal_path.push_back( optimal_path[i] );
    }

    // Reconstruct the configuration trajectory
    Eigen::VectorXd q_begin = Eigen::VectorXd::Zero(robot_model->getDimQ());
    Eigen::VectorXd q_end = Eigen::VectorXd::Zero(robot_model->getDimQ());

    // Create s vector as function of i.
    s_traj.clear();
    double s_start = 0.0;
    for(int i = 1; i < forward_order_optimal_path.size(); i++){
      // Cast Pointers
      current_ = static_pointer_cast<LMVertex>(forward_order_optimal_path[i]);
      parent_ = static_pointer_cast<LMVertex>(current_->parent);

      // Get the current and total delta_s 
      delta_s =  (current_->s - parent_->s);
      s_start = parent_->s;
      // Populate s vector with linear change
      for(int j = 0; j < N_size_per_edge; j++){
        s_traj.push_back(s_start + (delta_s / static_cast<double>(N_size_per_edge))*j); 
        // std::cout << "s_start + (delta_s / static_cast<double>(N_size_per_edge))*j = " << s_start + (delta_s / static_cast<double>(N_size_per_edge))*j << std::endl;
      }
    }

    // Debug printouts
    // std::cout << "forward_order_optimal_path.size() = " << forward_order_optimal_path.size() << std::endl;
    // std::cout << "N_size_per_edge * (forward_order_optimal_path.size() - 1) " << N_size_per_edge * (forward_order_optimal_path.size() - 1) << std::endl;
    // std::cout << "s_traj.size() = " << s_traj.size() << std::endl;

    // Initialize the full trajectory size
    double dt_dummy = 1e-3;
    path_traj_q_config.set_dim_N_dt(robot_model->getDimQ(), N_size_per_edge*(optimal_path.size()-1), dt_dummy);

    // We have to go through the optimal path order and identify if it was a manipulation or locomanipulation trajectory
    std::vector<int> current_edge_sequence;
    std::vector< std::vector<int> > edge_sequences;
    int edge_type = EDGE_TYPE_LOCOMANIPULATION;
    int prev_edge_type = EDGE_TYPE_LOCOMANIPULATION;
    for(int i = 1; i < forward_order_optimal_path.size(); i++){
      current_ = static_pointer_cast<LMVertex>(forward_order_optimal_path[i]);
      parent_ = static_pointer_cast<LMVertex>(current_->parent);

      // Store the previous edge type
      prev_edge_type = edge_type;

      // Check edge type
      if ( edgeHasStepTaken(parent_, current_, LEFT_FOOTSTEP) || (edgeHasStepTaken(parent_, current_, RIGHT_FOOTSTEP)) ) {
        // Current edge is a locomanipulation
        edge_type = EDGE_TYPE_LOCOMANIPULATION;
      }else{
        // Current edge is a manipulation
        edge_type = EDGE_TYPE_MANIPULATION;
      }

      // If the sequence has nonzero elements, check if the current edge is different from the previous edge.
      if (current_edge_sequence.size() > 0){
        // If the current edge type is different from the previous edge type, we have a new sequence
        if (edge_type != prev_edge_type){
          // store the sequence
          edge_sequences.push_back(current_edge_sequence);
          // start a new sequence
          current_edge_sequence.clear();
        }
      }
      // Store the edge type to the current sequence
      current_edge_sequence.push_back(edge_type);
      // If we are at the very end, store the current edge sequence
      if (i == forward_order_optimal_path.size() - 1){
        edge_sequences.push_back(current_edge_sequence);
      }
    }

    // std::cout << "edge_sequences.size() = " << edge_sequences.size() << std::endl;

    // Construct the trajectory based on the edge sequences.
    int N_sub_total = 0;
    int edge_number_offset = 0;
    int i_run = 0;

    for (int i = 0; i < edge_sequences.size(); i++){
      // std::cout << "edge_sequences[" << i << "].size() = " << edge_sequences[i].size() << std::endl;

      // Set the discretization value
      N_sub_total = edge_sequences[i].size()*N_size_per_edge;

      // Initialize the config trajectory discretization
      ctg->initializeDiscretization(N_sub_total);    

      // Populate desired hand configurations 
      // TO DO: Make this general for left, right or both.
      // std::cout << "setting hand poses" << std::endl;
      for (int j = 0; j < N_sub_total; j++){
        // Get the desired pose
        f_s->getPose(s_traj[j + edge_number_offset*N_size_per_edge], tmp_pos, tmp_ori);
        // Set the desired trajectory
        ctg->traj_SE3_right_hand.set_pos(j, tmp_pos, tmp_ori);
      }


      // Set the current vertex and its parent
      current_ = static_pointer_cast<LMVertex>(forward_order_optimal_path[edge_number_offset + 1]);
      parent_ = static_pointer_cast<LMVertex>(current_->parent);


      // std::cout << "creating footsteps" << std::endl;
      // Clear the footsteps
      input_footstep_list.clear();      
      // Check if this is a locomanipulation edge. 
      if (edge_sequences[i][0] == EDGE_TYPE_LOCOMANIPULATION){
        // If so, populate footstep list
        for(int j = 0; j < (edge_sequences[i].size()); j++){
          std::cout << "j" << j << std::endl;
          // std::cout << "edge_number_offset" << edge_number_offset << std::endl;

          std::shared_ptr<LMVertex> local_current = static_pointer_cast<LMVertex>(forward_order_optimal_path[j + edge_number_offset + 1]);
          std::shared_ptr<LMVertex> local_parent = static_pointer_cast<LMVertex>(local_current->parent);;        

          // Check if a left or right footstep is taken
          if (edgeHasStepTaken(local_parent, local_current, LEFT_FOOTSTEP)) {
            input_footstep_list.push_back(local_current->left_foot);
            local_current->left_foot.printInfo();
          }else if (edgeHasStepTaken(local_parent, local_current, RIGHT_FOOTSTEP)){
            input_footstep_list.push_back(local_current->right_foot);        
            local_current->right_foot.printInfo();
          }
        }

      }
      std::cout << "computing the configuration" << std::endl;

      // Compute configuration trajectory for this edge sequence
      bool convergence = false;
      convergence = ctg->computeConfigurationTrajectory(parent_->q_init, input_footstep_list);

      std::cout << "convergence = " << convergence << std::endl;

      // Get the final configuration and set to current
      // std::cout << "getting the final configuration" << std::endl;
      ctg->traj_q_config.get_pos(ctg->getDiscretizationSize() - 1, q_end);
      current_->setRobotConfig(q_end);  

      // Store q trajectory configuration
      // Check for convergence.
      if (convergence){
        for(int j = 0; j < N_sub_total; j++){
          // Get the configuration at this local trajectory
          ctg->traj_q_config.get_pos(j, q_tmp);
          // Store the trajectory to the global path
          path_traj_q_config.set_pos(i_run + j, q_tmp);        
        }
        // Update i_run
        i_run += N_sub_total;        
      }else{
        return false;
      }

      // Update the edge number offset
      edge_number_offset += edge_sequences[i].size();
    }

    // To do: update the dt properly for each segment of the trajectory. For now  set a desired dt for visualization
    path_traj_q_config.set_dt(ctg->wpg.traj_pos_com.get_dt() );  // seconds   
   // path_traj_q_config.set_dt(0.05);  // seconds   

    // Reset discretization
    ctg->initializeDiscretization(N_size_per_edge);

    return true;

  }


  bool LocomanipulationPlanner::reconstructConfigurationTrajectory(){
    // Iterate through optimal path backwards. to construct the forward path
    forward_order_optimal_path.clear();
    for(int i = (optimal_path.size() - 1); i >= 0; i--){
      forward_order_optimal_path.push_back( optimal_path[i] );
    }

    // Create s vector as function of i.
    s_traj.clear();
    double s_start = 0.0;
    for(int i = 1; i < forward_order_optimal_path.size(); i++){
      // Cast Pointers
      current_ = static_pointer_cast<LMVertex>(forward_order_optimal_path[i]);
      parent_ = static_pointer_cast<LMVertex>(current_->parent);

      // Get the current and total delta_s 
      delta_s =  (current_->s - parent_->s);
      s_start = parent_->s;
      // Populate s vector with linear change
      for(int j = 0; j < N_size_per_edge; j++){
        s_traj.push_back(s_start + (delta_s / static_cast<double>(N_size_per_edge))*j); 
        // std::cout << "s_start + (delta_s / static_cast<double>(N_size_per_edge))*j = " << s_start + (delta_s / static_cast<double>(N_size_per_edge))*j << std::endl;
      }
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

      // Check classifier result for this transition
      bool transition_possible = true;
      if (use_classifier){
        double feas_score = getFeasibility(parent_, current_);
        if (print_classifier_results){
          std::cout << "Feasibility Score = " << feas_score << std::endl;
        }
        // If score is below the threshold, this transition is not possible
        if (feas_score < feasibility_threshold){
          transition_possible = false;
        }
      }

      // Update the input footstep list
      input_footstep_list.clear();
      // Check if a left or right footstep is taken
      if (edgeHasStepTaken(parent_, current_, LEFT_FOOTSTEP)) {
        input_footstep_list.push_back(current_->left_foot);
        current_->left_foot.printInfo();
      }else if (edgeHasStepTaken(parent_, current_, RIGHT_FOOTSTEP)){
        input_footstep_list.push_back(current_->right_foot);        
        current_->right_foot.printInfo();
      }

      // Set initial configuration
      delta_s =  (current_->s - parent_->s);

      bool convergence = false;
      convergence = ctg->computeConfigurationTrajectory(f_s, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                                                  parent_->s, delta_s, 
                                                                  parent_->q_init, 
                                                                 input_footstep_list);
      // Get the final configuration and set to current
      // std::cout << "getting the final configuration" << std::endl;
      ctg->traj_q_config.get_pos(ctg->getDiscretizationSize() - 1, q_end);
      current_->setRobotConfig(q_end);

      // Store the data if the classifier made a mistake during reconstruction.
      // only look at mistakes in which the s variable did not move as this is the only thing we can learn
      if (use_classifier && (!edgeHasSVarMoved(parent_, current_)) ){
        if ((classifier_store_mistakes_during_reconstruction) || (classifier_store_mistakes)){
            // False Positive.
            if ((transition_possible) && (!convergence)){
              std::cout << "  False Positive" << std::endl;
              storeTransitionDatawithTaskSpaceInfo(parent_, false);
            } 
            // False Negatives
            else if ((!transition_possible) && (convergence)){
              std::cout << "  False Negative" << std::endl;
              storeTransitionDatawithTaskSpaceInfo(parent_, true);
            }
        }        
      }

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
    // path_traj_q_config.set_dt(0.025);  // seconds   
    path_traj_q_config.set_dt(ctg->wpg.traj_pos_com.get_dt() );  // seconds   

    std::cout << "Configuration Path Reconstruction success " << std::endl;
    return true;

  }

  void LocomanipulationPlanner::setStanceFoot(const shared_ptr<LMVertex> & from_node, const int robot_side){
    if (robot_side == LEFT_FOOTSTEP){
      nn_stance_origin = CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE;
      feasibility_stance_foot_pos = from_node->left_foot.position;
      feasibility_stance_foot_ori = from_node->left_foot.orientation;
      // Set the rotation matrix
      R_stance_origin = from_node->left_foot.R_ori;
    }else if (robot_side == RIGHT_FOOTSTEP){
      nn_stance_origin = CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE; 
      feasibility_stance_foot_pos = from_node->right_foot.position;
      feasibility_stance_foot_ori = from_node->right_foot.orientation;
      // Set the rotation matrix
      R_stance_origin = from_node->right_foot.R_ori;
    }
      R_stance_origin_transpose = R_stance_origin.transpose();
  }

  void LocomanipulationPlanner::setSwingFoot(const shared_ptr<LMVertex> & from_node, const shared_ptr<LMVertex> & to_node, const int robot_side){
    if (robot_side == LEFT_FOOTSTEP){
      // Set the left foot swing start and landing poses
      nn_swing_foot_start_pos = from_node->left_foot.position;
      nn_swing_foot_start_ori = from_node->left_foot.orientation;
      nn_landing_foot_pos = to_node->left_foot.position;
      nn_landing_foot_ori = to_node->left_foot.orientation;
    }else if (robot_side == RIGHT_FOOTSTEP){
      // Set the right foot swing start and landing poses
      nn_swing_foot_start_pos = from_node->right_foot.position;
      nn_swing_foot_start_ori = from_node->right_foot.orientation;
      nn_landing_foot_pos = to_node->right_foot.position;
      nn_landing_foot_ori = to_node->right_foot.orientation;
    }
  }

  // Assumes that the stance frame has already been set
  void LocomanipulationPlanner::setHandPoses(double s_value){
    // TODO: manipulation function f_s should be setting the manipulation type
    nn_manipulation_type = CONTACT_TRANSITION_DATA_RIGHT_HAND; 


    if (nn_manipulation_type == CONTACT_TRANSITION_DATA_RIGHT_HAND){
      // Get the hand pose from the manipulation function
      f_s->getPose(s_value, nn_right_hand_start_pos, nn_right_hand_start_ori);

      // Convert Right Hand to Stance Frame
      convertWorldToStanceOrigin(nn_right_hand_start_pos, nn_right_hand_start_ori, tmp_pos, tmp_ori);
      nn_right_hand_start_pos = tmp_pos; nn_right_hand_start_ori = tmp_ori;

      // Zero out the left hand
      nn_left_hand_start_pos.setZero();
      nn_left_hand_start_ori.setIdentity();
    }else if (nn_manipulation_type == CONTACT_TRANSITION_DATA_LEFT_HAND){
      // Get the hand pose from the manipulation function
      f_s->getPose(s_value, nn_left_hand_start_pos, nn_left_hand_start_ori);      

      // Convert Left Hand to Stance Frame
      convertWorldToStanceOrigin(nn_left_hand_start_pos, nn_left_hand_start_ori, tmp_pos, tmp_ori);
      nn_left_hand_start_pos = tmp_pos; nn_left_hand_start_ori = tmp_ori;

      nn_right_hand_start_pos.setZero();
      nn_right_hand_start_ori.setIdentity();
    }else if (nn_manipulation_type == CONTACT_TRANSITION_DATA_BOTH_HANDS){
      // not implemented
      // f_s->getPose(s_value, nn_left_hand_start_pos, nn_left_hand_start_ori,
      //                            nn_right_hand_start_pos, nn_right_hand_start_ori);      

      // Convert Right Hand to Stance Frame
      convertWorldToStanceOrigin(nn_right_hand_start_pos, nn_right_hand_start_ori, tmp_pos, tmp_ori);
      nn_right_hand_start_pos = tmp_pos; nn_right_hand_start_ori = tmp_ori;
      // Convert Left Hand to Stance Frame
      convertWorldToStanceOrigin(nn_left_hand_start_pos, nn_left_hand_start_ori, tmp_pos, tmp_ori);
      nn_left_hand_start_pos = tmp_pos; nn_left_hand_start_ori = tmp_ori;
 
    }
  }

  void LocomanipulationPlanner::setStanceSwingPelvisNNVariables(const shared_ptr<LMVertex> & from_node, const shared_ptr<LMVertex> & to_node, int robot_side){
    // Add constant height offset for the pelvis z direction from the midfoot:
    Eigen::Vector3d pelvis_dz(0.0, 0.0, nn_starting_pelvis_height);
    nn_pelvis_pos = from_node->mid_foot.position + from_node->mid_foot.R_ori*pelvis_dz;
    // Set starting pelvis ori to the midfoot orientation
    nn_pelvis_ori = from_node->mid_foot.orientation;

    // Set Variables for a left footstep
    if (robot_side == LEFT_FOOTSTEP){
      setSwingFoot(from_node, to_node, LEFT_FOOTSTEP);
      setStanceFoot(from_node, RIGHT_FOOTSTEP);      
    } // Otherwise set it for a right footstep

    if (robot_side == RIGHT_FOOTSTEP){
      setSwingFoot(from_node, to_node, RIGHT_FOOTSTEP);
      setStanceFoot(from_node, LEFT_FOOTSTEP);
    }

    // Convert Pelvis Pose to Stance Frame
    convertWorldToStanceOrigin(nn_pelvis_pos, nn_pelvis_ori, tmp_pos, tmp_ori);
    nn_pelvis_pos = tmp_pos; nn_pelvis_ori = tmp_ori;
    // Convert Starting Swing to Stance Frame
    convertWorldToStanceOrigin(nn_swing_foot_start_pos, nn_swing_foot_start_ori, tmp_pos, tmp_ori);
    nn_swing_foot_start_pos = tmp_pos; nn_swing_foot_start_ori = tmp_ori;
    // Convert Landing Foot to Stance Frame
    convertWorldToStanceOrigin(nn_landing_foot_pos, nn_landing_foot_ori, tmp_pos, tmp_ori);
    nn_landing_foot_pos = tmp_pos; nn_landing_foot_ori = tmp_ori;      

  }

  void LocomanipulationPlanner::computeHandTrajectoryFeasibility(const shared_ptr<LMVertex> & from_node, const shared_ptr<LMVertex> & to_node){
    // Check feasibility for each s along the trajectory 
    double s_local = 0.0;
    nn_delta_s =  (to_node->s - from_node->s);
    double s_local_min = 0.0;

    // For each s, check the neural network for feasibility. 
    for(int i = 0; i < (N_s+1); i++){
      s_local = from_node->s + (static_cast<double>(i)/static_cast<double>(N_s))*nn_delta_s;
      // Update hand/s pose/s and convert to stance frame
      setHandPoses(s_local);

      // Query the neural network classifier 
      nn_prediction_score = getClassifierResult();

      // Keep track of the lowest result
      if (nn_prediction_score < nn_feasibility_score){
        s_local_min = s_local;
        nn_feasibility_score = nn_prediction_score;
      }
    }
    // Set the hand poses to the minimum classifier result.
    setHandPoses(s_local_min);

  }

  // compute the feasibility score depending on edge type
  double LocomanipulationPlanner::getFeasibility(const shared_ptr<LMVertex> & from_node, const shared_ptr<LMVertex> & to_node){
    // std::cout << "Computing feasibility" << std::endl;
    bool left_step_taken = edgeHasStepTaken(from_node, to_node, LEFT_FOOTSTEP);
    bool right_step_taken = edgeHasStepTaken(from_node, to_node, RIGHT_FOOTSTEP);

    bool step_taken = left_step_taken || right_step_taken;
    bool s_var_moved = edgeHasSVarMoved(from_node, to_node);

    // Reset feasibility and prediction scores
    nn_feasibility_score = 1.0;
    nn_prediction_score = 0.0;

    // Cartesian Product between whether or not there are steps and whether or not s has moved.
    //    do not consider the case when there are no footsteps and no s changes 
    // cases = {step_taken, no_steps} X {s_moved, s_constant} \ {(no_steps, s_constant)}

    // Case 1: Step has been taken and the s variable did not move-----------------------------------------------------
    if ((step_taken) && (!s_var_moved)){
      // Set NN variables based on which step was taken
      if (left_step_taken){
        setStanceSwingPelvisNNVariables(from_node, to_node, LEFT_FOOTSTEP);        
      }else if (right_step_taken){
        setStanceSwingPelvisNNVariables(from_node, to_node, RIGHT_FOOTSTEP);                
      }

      // set the hand pose to the start
      setHandPoses(from_node->s);
      // Query the neural network classifier as normal
      nn_prediction_score = getClassifierResult();
      nn_feasibility_score = nn_prediction_score;
    }

    // Case 2: Step has been taken and the s variable has moved--------------------------------------------------------  
    else if ((step_taken) && (s_var_moved)){ 
      // Set NN variables based on which step was taken
      if (left_step_taken){
        setStanceSwingPelvisNNVariables(from_node, to_node, LEFT_FOOTSTEP);        
      }else if (right_step_taken){
        setStanceSwingPelvisNNVariables(from_node, to_node, RIGHT_FOOTSTEP);                
      }

      // Compute min feasibility score from the hand trajectory while taking a step
      computeHandTrajectoryFeasibility(from_node, to_node);

    }

    // Case 3: No step is taken and the s variable has moved-----------------------------------------------------------
    else if ((!step_taken) && (s_var_moved)){
      // We want to compute the locomanipulability of this hand configuration by asking whether or not 
      // it's possible to take footstep in place while keeping the hand in place. Do this over the discretization of s

      // Compute min feasibility score of the hand trajectory while taking an in place right footstep
      setStanceSwingPelvisNNVariables(from_node, from_node, RIGHT_FOOTSTEP);        
      computeHandTrajectoryFeasibility(from_node, to_node);

      // Using the same feasibility score, 
      // Compute min feasibility score of the hand trajectory while taking an in place left footstep
      setStanceSwingPelvisNNVariables(from_node, from_node, LEFT_FOOTSTEP);
      computeHandTrajectoryFeasibility(from_node, to_node);
    }else{
      std::cout << "Error. No step has been taken and the s variable did not move. Something must be wrong with the neighbor generation or the edge type transition checks " << std::endl;
    }

    return nn_feasibility_score;
  }


  void LocomanipulationPlanner::setClassifierClient(ros::ServiceClient & classifier_client_in){
    classifier_client = classifier_client_in;
    print_classifier_results = true;
    use_classifier = true;
  }

  void LocomanipulationPlanner::setNeuralNetwork(std::shared_ptr<NeuralNetModel> nn_model_in, const Eigen::VectorXd & nn_mean_in, const Eigen::VectorXd & nn_std_dev_in){
    nn_model = nn_model_in;
    nn_mean = nn_mean_in;
    nn_std = nn_std_dev_in;
    use_classifier = true;
    enable_cpp_nn = true;

    cpp_nn_rawDatum = Eigen::VectorXd::Zero(32);
    cpp_nn_normalized_datum = Eigen::VectorXd::Zero(32);
    cpp_nn_input_size = 1;
    cpp_nn_data_input = Eigen::MatrixXd::Zero(cpp_nn_input_size, 32);
    cpp_nn_pred = Eigen::MatrixXd::Zero(cpp_nn_input_size,1);
  }

  // Locomanipulation gscore
  double LocomanipulationPlanner::gScore(const shared_ptr<Node> current, const shared_ptr<Node> neighbor){
    current_ = std::static_pointer_cast<LMVertex>(current);
    neighbor_ = std::static_pointer_cast<LMVertex>(neighbor);

    // Get Hand position and orientation at the neighbor's s.
    f_s->getPose(neighbor_->s, tmp_pos, tmp_ori);    
    
    // Compute the estimated left and right foot landing locations
    lf_guess_pos = tmp_ori.toRotationMatrix()*lf_pos_wrt_hand + tmp_pos;
    lf_guess_ori = tmp_ori*lf_ori_wrt_hand;

    rf_guess_pos = tmp_ori.toRotationMatrix()*rf_pos_wrt_hand + tmp_pos;
    rf_guess_ori = tmp_ori*rf_ori_wrt_hand;
    
    double lf_distance_cost = (lf_guess_pos - neighbor_->left_foot.position).norm() + fabs( getAngle(lf_guess_ori) - getAngle(neighbor_->left_foot.orientation) );
    double rf_distance_cost = (rf_guess_pos - neighbor_->right_foot.position).norm() + fabs( getAngle(rf_guess_ori) - getAngle(neighbor_->right_foot.orientation) );


    double s_cost = w_s*(neighbor_->s - current_->s);
    // Midfoot cost
    // double distance_cost = w_distance*((neighbor_->mid_foot.position - current_->mid_foot.position).norm() 
    //                                     + fabs(getAngle(neighbor_->mid_foot.orientation) - getAngle(current_->mid_foot.orientation)) );

    // Foot landing cost on guess
    double distance_cost = w_distance*(lf_distance_cost + rf_distance_cost);

   
    bool left_step_taken = edgeHasStepTaken(current_, neighbor_, LEFT_FOOTSTEP);
    bool right_step_taken = edgeHasStepTaken(current_, neighbor_, RIGHT_FOOTSTEP);

    double step_cost = 0.0;
    if (left_step_taken || right_step_taken){
      step_cost = w_step;
    }
    
    double transition_distance_cost = 0.0;
    if (left_step_taken){
      transition_distance_cost = w_transition_distance*((neighbor_->left_foot.position - current_->left_foot.position).norm() 
                                              + fabs(getAngle(neighbor_->left_foot.orientation) - getAngle(current_->left_foot.orientation)) );
    }else if (right_step_taken){
      transition_distance_cost = w_transition_distance*((neighbor_->right_foot.position - current_->right_foot.position).norm() 
                                              + fabs(getAngle(neighbor_->right_foot.orientation) - getAngle(current_->right_foot.orientation)) ); 
    }

    double feasibility_cost = 0.0; 
    // if (use_classifier){
    //   feasibility_cost =  w_feasibility*(1.0 - getFeasibility(current_, neighbor_));      
    // }

    double delta_g = s_cost + distance_cost + step_cost + transition_distance_cost + feasibility_cost;

    return delta_g;    
  }
  // Locomanipulation heuristic 
  double LocomanipulationPlanner::heuristicCost(const shared_ptr<Node> neighbor,const shared_ptr<Node> goal){
    neighbor_ = static_pointer_cast<LMVertex>(neighbor);
    goal_ = static_pointer_cast<LMVertex>(goal);

    double s_cost = w_s*(goal_->s - neighbor_->s);

    // // Get Hand position and orientation at the neighbor's s.
    // f_s->getPose(goal_->s, tmp_pos, tmp_ori);    
 
    // // Compute the estimated left and right foot landing locations
    // lf_guess_pos = tmp_ori.toRotationMatrix()*lf_pos_wrt_hand + tmp_pos;
    // lf_guess_ori = tmp_ori*lf_ori_wrt_hand;

    // rf_guess_pos = tmp_ori.toRotationMatrix()*rf_pos_wrt_hand + tmp_pos;
    // rf_guess_ori = tmp_ori*rf_ori_wrt_hand;
    
    // double lf_distance_cost = (lf_guess_pos - neighbor_->left_foot.position).norm() + fabs( getAngle(lf_guess_ori) - getAngle(neighbor_->left_foot.orientation) );
    // double rf_distance_cost = (rf_guess_pos - neighbor_->right_foot.position).norm() + fabs( getAngle(rf_guess_ori) - getAngle(neighbor_->right_foot.orientation) );


    // // Mid feet frame heuristic
    // // double distance_cost = w_distance*((goal_->mid_foot.position - neighbor_->mid_foot.position).norm() 
    // //                                     + fabs(getAngle(goal_->mid_foot.orientation) - getAngle(neighbor_->mid_foot.orientation)) );

    // // Footstep landing heuristic
    // double distance_cost = w_distance*(lf_distance_cost + rf_distance_cost);

    // Linear distance to the manipulation goal. if w_heuristic = 1.0, we get the true A* result 
    // return w_heuristic*(s_cost + distance_cost); 
    return w_heuristic*(s_cost);
  }

  // Locomanipulation whether or not the goal was reached
  bool LocomanipulationPlanner::goalReached(shared_ptr<Node> current_node, shared_ptr<Node> goal){
    current_ = static_pointer_cast<LMVertex>(current_node);  
    goal_ = static_pointer_cast<LMVertex>(goal);

    // check if s is within 0.05 of goal_s
    std::cout << "  Goal Check (goal_s, current_s) = (" << goal_->s << ", " << current_->s << ")" << std::endl;
    bool s_satisfaction = (fabs(goal_->s - current_->s) <= goal_tol);
    bool convergence = false;

    // TODO: Change condition if we are using the neural network.

    // If s is within tolerance, actually check if we can perform this trajectory
    if (s_satisfaction){
      // std::cout << "within goal tolerance checking for convergence" << std::endl;
      parent_ = static_pointer_cast<LMVertex>(current_->parent);
      delta_s =  (current_->s - parent_->s);
      // Update the input footstep list
      input_footstep_list.clear();
      if (edgeHasStepTaken(parent_, current_, LEFT_FOOTSTEP)) {
        input_footstep_list.push_back(current_->left_foot);
      }else if (edgeHasStepTaken(parent_, current_, RIGHT_FOOTSTEP)){
        input_footstep_list.push_back(current_->right_foot);        
      }

      // Check the classifier result
      bool transition_possible = true;
      if (use_classifier){
        double feas_score = getFeasibility(parent_, current_);
        if (print_classifier_results){
          std::cout << "  GC Feasibility Score = " << feas_score << std::endl;
        }
        // If the score is less than the treshold
        if (feas_score < feasibility_threshold){
          transition_possible = false;
          // proceed with returning false if we are not storing classifier mistakes
          if (!classifier_store_mistakes) {
            return false;
          }
        }
      }

      // If we do not trust the classifier, run a configuration checl
      if (!trust_classifier){
        convergence = ctg->computeConfigurationTrajectory(f_s, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                                                    parent_->s, delta_s, 
                                                                    parent_->q_init, 
                                                                   input_footstep_list);

        // if store mistakes
        if ((use_classifier) && (classifier_store_mistakes)){
            // Store the data if the classifier made a mistake.
            // False Positive.
            if ((transition_possible) && (!convergence)){
              std::cout << "  False Positive" << std::endl;
              storeTransitionDatawithTaskSpaceInfo(parent_, false);
            } 
            // False Negatives
            else if ((!transition_possible) && (convergence)){
              std::cout << "  False Negative" << std::endl;
              storeTransitionDatawithTaskSpaceInfo(parent_, true);
            }
        }

        if (convergence){
          ctg->traj_q_config.get_pos(ctg->getDiscretizationSize() - 1, q_tmp);
          // std::cout << "goal q_tmp = " << q_tmp.transpose() << std::endl;
          current_->setRobotConfig(q_tmp);       
        }
      }
      else{
        convergence = true;
      }// End classifier trust check

    }

    std::cout << "  Goal Reached: " << ((s_satisfaction && convergence) ? "True" : "False") << std::endl;
    return (s_satisfaction && convergence);
  }


  void LocomanipulationPlanner::generateDiscretization(){
    // if (classifier_store_mistakes)
    //   delta_s_vals = {0.01, 0.04};
    // else{
    //   delta_s_vals = {0.01, 0.04};
    // }
    delta_s_vals = {0.01, 0.04, 0.08};//{0.01, 0.04, 0.08, 0.10};

    //number of bins for the local lattice
    int num_x_lattice_pts = int(abs(2*max_lattice_translation)/dx) + 1;
    int num_y_lattice_pts = int(abs(2*max_lattice_translation)/dy) + 1;
    int num_theta_lattice_pts = int(abs(max_lattice_theta - min_lattice_theta)/dtheta) + 1;

    // create x coordinate local lattice from -max_lattice_translation to max_lattice_translation spaced by dx
    dx_vals.clear();
    for(int i = 0; i < num_x_lattice_pts; i++){
      dx_vals.push_back( i*dx - max_lattice_translation);
     //std::cout << "x_lattice_pt " << i << " " << dx_vals[i] << std::endl;

    }
    // create y coordinate local lattice from -max_lattice_translation to max_lattice_translation spaced by dx
    dy_vals.clear();
    for(int i = 0; i < num_y_lattice_pts; i++){
      dy_vals.push_back( i*dy - max_lattice_translation);
      //std::cout << "y_lattice_pt " << i << " " << dy_vals[i] << std::endl;
    }
    // create dtheta coordinate local lattice from -min_lattice_theta to max_lattice_theta spaced by dtheta
    dtheta_vals.clear();
    for(int i = 0; i < num_theta_lattice_pts; i++){
      dtheta_vals.push_back( i*dtheta - max_lattice_theta);
     //std::cout << "dtheta_lattice_pt " << i << " " << dtheta_vals[i] << std::endl;
    }

    int max_neighbors_size = delta_s_vals.size() + 2*delta_s_vals.size()*dx_vals.size()*dy_vals.size()*dtheta_vals.size();
    std::cout << "Maximum possible neighbor size: " << max_neighbors_size << std::endl;

    // Preallocate space
    neighbors.reserve(max_neighbors_size);
  }


  // Ensures that is is within 0.0 and goal_s
  double LocomanipulationPlanner::clamp_s_variable(const double s_in){
    if (s_in >= goal_s){
      return (goal_s - 1e-4); 
    }else if (s_in <= 0.0){
      return 0.0;
    }else{
      return s_in;
    }
  }

  // Function adds non-footstep neighbors to the list of neighbors using the current node
  void LocomanipulationPlanner::generateNonFootstepNeighbors(){
    // Generate no step neighbors. Only varies with delta_s
    for(int i = 0; i < delta_s_vals.size(); i++){
     // Create the neighbor only if the proposed s is not close to 0.0.
      if ( fabs(delta_s_vals[i]) > 1e-6){
        // ensure that s is bounded between 0 and 1.
        shared_ptr<Node> neighbor (std::make_shared<LMVertex>( clamp_s_variable(current_->s + delta_s_vals[i]), current_->left_foot, current_->right_foot));
        // Update the neighbor (probably make this a function to call)
        neighbor_change = static_pointer_cast<LMVertex>(neighbor);
        neighbor_change->parent = static_pointer_cast<Node>(current_);
        // neighbor = static_pointer_cast<Node>(neighbor_change);

        // TODO: Add feasibility check here and only add to neighbors if it's greater than our threshold

        if (trust_classifier){
          // Skip this neighbor if it is not within the threshold
          double feas_score = getFeasibility(current_, neighbor_change);
          if (feas_score < feasibility_threshold){
            continue;
          }
        }

        neighbors.push_back(neighbor);
      }
    }
    
  }

  void LocomanipulationPlanner::generateFootstepNeighbors(int footstep_side){
    // std::cout << "Generating " << (footstep_side == RIGHT_FOOTSTEP ? "right" : "left") << " landing foot neighbors" << std::endl;

    //  Initialize footstep landing location object and stance foot location.
    //  These are both in the planner origin frame.
    Footstep landing_foot, stance_foot;
    Eigen::Vector3d   landing_pos, delta_translate; landing_pos.setZero(); delta_translate.setZero();
    Eigen::Quaterniond landing_quat, delta_quat;  landing_quat.setIdentity(); delta_quat.setIdentity();

    // Generate footstep neighbors depending on the stance foot
    // delta_s, dx, dy, dtheta

    // Set stance foot depending on the landing foot and convert it to the planner origin frame
    if (footstep_side == RIGHT_FOOTSTEP){
      convertWorldToPlannerOrigin(current_->left_foot.position, current_->left_foot.orientation, tmp_pos, tmp_ori);
      stance_foot.setPosOriSide(tmp_pos, tmp_ori, LEFT_FOOTSTEP);      
    }else if (footstep_side == LEFT_FOOTSTEP){
      convertWorldToPlannerOrigin(current_->right_foot.position, current_->right_foot.orientation, tmp_pos, tmp_ori);
      stance_foot.setPosOriSide(tmp_pos, tmp_ori, RIGHT_FOOTSTEP);      
    }

    int counter = 0;

    // Get the hand pose from the manipulation function
    Eigen::Vector3d tmp_hand_pos; 
    Eigen::Quaterniond tmp_hand_ori;
    f_s->getPose(current_->s, tmp_hand_pos, tmp_hand_ori);

    double feas_score = 0.0;
    for(int i = 0; i < delta_s_vals.size(); i++){
      for(int j = 0; j < dx_vals.size(); j++){
        for (int k = 0; k < dy_vals.size(); k++){
          for(int m = 0; m < dtheta_vals.size(); m++){
            // Prepare dx, dy, dtheta
            delta_translate[0] = dx_vals[j];
            delta_translate[1] = dy_vals[k];

            delta_quat.x() = 0.0;
            delta_quat.y() = 0.0;
            delta_quat.z() = sin(dtheta_vals[m]/2.0);
            delta_quat.w() = cos(dtheta_vals[m]/2.0);

            // Compute delta x,y, theta w.r.t the stance
            landing_pos = stance_foot.position + delta_translate;

            // Skip if this landing position is greater than the maximum lattice radius
            if (landing_pos.norm() >= max_lattice_radius){
              // std::cout << " potential neighbor is greater than the max lattice radius" << std::endl;
              continue;
            }
            landing_quat = stance_foot.orientation*delta_quat;


            // Check if the landing foot is within the kinematic bounds w.r.t. the stance foot
            if (withinKinematicBounds(stance_foot, landing_pos, landing_quat)){
              // std::cout << "accepted" << std::endl;

              // std::cout << "planner frame potential landing_foot_pos = " << landing_pos.transpose() << std::endl;
              // std::cout << "planner frame potential landing_foot_ori = " << getAngle(landing_quat) << std::endl;             
              // std::cout << "world frame potential landing_foot_pos = " << tmp_pos.transpose() << std::endl;
              // std::cout << "world frame potential landing_foot_ori = " << getAngle(tmp_ori) << std::endl;             

              // Convert landing location back to the world frame
              convertPlannerToWorldOrigin(landing_pos, landing_quat, tmp_pos, tmp_ori);
              landing_foot.setPosOriSide(tmp_pos, tmp_ori, footstep_side);                

              // TODO: Check if hand position is within the kinematic bounds from the pelvis position.
              if ((landing_foot.position - tmp_hand_pos).norm() >= max_foot_to_hand_radius) {
                std::cout << "  potential neighbor has foot-to-hand-distance greater than the kinematic limits" << std::endl;
                continue;
              } 



              shared_ptr<Node> neighbor;
              // Landing foot should go to the correct footstep. the stance foot remains unchanged
              if (footstep_side == RIGHT_FOOTSTEP){
                 neighbor = std::make_shared<LMVertex>(clamp_s_variable(current_->s + delta_s_vals[i]), 
                                                                      current_->left_foot, 
                                                                      landing_foot);
              }else{
                 neighbor = std::make_shared<LMVertex>(clamp_s_variable(current_->s + delta_s_vals[i]), 
                                                                      landing_foot,
                                                                      current_->right_foot);
              }
              // Update the neighbor's parent
              neighbor_change = static_pointer_cast<LMVertex>(neighbor);
              neighbor_change->parent = static_pointer_cast<Node>(current_);

              // TODO: Add feasibility check here and only add to neighbors if it's greater than our threshold

              if (trust_classifier){
                // Skip this neighbor if it is not within the threshold
                double feas_score = getFeasibility(current_, neighbor_change);
                if (feas_score < feasibility_threshold){
                  continue;
                }
              }

              // Add landing foot 
              neighbors.push_back(neighbor);

              // Increment counter
              counter++;
            } // End kinematic bounds check
          } // End delta theta for loop
        } // End delta y for loop
      } // End delta x for loop
    }  // End delta s for loop

    // std::cout << "number of landing foot neighbors = " << counter << std::endl;


  }


  std::vector< std::shared_ptr<Node> > LocomanipulationPlanner::getNeighbors(shared_ptr<Node> & current){
    std::cout << "Getting Neighbors" << std::endl;
    current_ = static_pointer_cast<LMVertex>(current);  
    // Clear previous neighbor list
    neighbors.clear();

    // Check if this is the first time get Neighbors is being evaluated. 
    bool convergence = false;
    if (first_node_evaluated){
      // std::cout << "Testing trajectory feasiblity for the current node" << std::endl;     

      parent_ = static_pointer_cast<LMVertex>(current_->parent);
      delta_s =  (current_->s - parent_->s);

      // Update the input footstep list
      input_footstep_list.clear();
      if (edgeHasStepTaken(parent_, current_, LEFT_FOOTSTEP)) {
        input_footstep_list.push_back(current_->left_foot);
      }else if (edgeHasStepTaken(parent_, current_, RIGHT_FOOTSTEP)){
        input_footstep_list.push_back(current_->right_foot);        
      }

      bool transition_possible = true;
      if (use_classifier){
        double feas_score = getFeasibility(parent_, current_);
        if (print_classifier_results){
          std::cout << "  Feasibility Score = " << feas_score << std::endl;
        }
        // If the score is less than the treshold, don't bother with the computation
        if (feas_score < feasibility_threshold){
          transition_possible = false;
          std::cout << "  Transition Possible: " << (transition_possible ? "True" : "False") << std::endl; 
          // If we are not storing possible mistakes, proceed with ignoring the computation check
          // if the s variable has moved, we cannot learn from this example so ignore the computation check
          // if ((!classifier_store_mistakes) || edgeHasSVarMoved(parent_, current_)){

          if (!classifier_store_mistakes){
            return neighbors;
          }
        }else{
          std::cout << "  Transition Possible: " << (transition_possible ? "True" : "False") << std::endl;           
        }      
      }

      // Start -- IF we don't trust the classifier, run a config trajectory check
      if (!trust_classifier){

        convergence = ctg->computeConfigurationTrajectory(f_s, CONFIG_TRAJECTORY_ROBOT_RIGHT_SIDE, 
                                                                    parent_->s, delta_s, 
                                                                    parent_->q_init, 
                                                                    input_footstep_list);

        std::cout << "  Converged: " << (convergence ? "True" : "False") << std::endl;       

        // Store the transition if we are using the classifier, storing the mistakes and
        // the s variable has not moved

        // if ((use_classifier) && (classifier_store_mistakes) && (!edgeHasSVarMoved(parent_, current_)) ){
        if (use_classifier && classifier_store_mistakes){
            // Store the data if the classifier made a mistake.
            // False Positive.
            if ((transition_possible) && (!convergence)){
              std::cout << "  False Positive" << std::endl;
              storeTransitionDatawithTaskSpaceInfo(parent_, false);
            } 
            // False Negatives
            else if ((!transition_possible) && (convergence)){
              std::cout << "  False Negative" << std::endl;
              storeTransitionDatawithTaskSpaceInfo(parent_, true);
            }

        }

        // If it converges, update the configuration of the current node
        if (convergence){
          // Get the final configuration.
          // std::cout << "Current node is feasible. Updating q_tmp" << std::endl;     
          ctg->traj_q_config.get_pos(ctg->getDiscretizationSize() - 1, q_tmp);
          // std::cout << "q_tmp = " << q_tmp.transpose() << std::endl;
          current_->setRobotConfig(q_tmp);
        }else{
          // If it does not converge, return an empty neighbor list       
          return neighbors;       
        }
      }
      // END ----

    }else{  
      // std::cout << "first node evaluated is true" << std::endl;
      first_node_evaluated = true;      

      // Generate neighbors with lazy evaluation ie: All neighbors are valid unless it has been attempted
      // std::cout << " -- gen. non footstep neighbors --" << std::endl;
      // generateNonFootstepNeighbors();
      std::cout << " -- gen. left footstep neighbors --" << std::endl;
      generateFootstepNeighbors(LEFT_FOOTSTEP);
      std::cout << " -- gen. right footstep neighbors --" << std::endl;
      generateFootstepNeighbors(RIGHT_FOOTSTEP);
      std::cout << "num neighbors generated: " << neighbors.size() << std::endl;

      return neighbors;
    }

    // std::cout << "generating neighbors for the current neighbor" << std::endl;

    // Generate neighbors with lazy evaluation ie: All neighbors are valid unless it has been attempted
    std::cout << " -- gen. non footstep neighbors --" << std::endl;
    generateNonFootstepNeighbors();
    // Force alternating footsteps
    parent_ = static_pointer_cast<LMVertex>(current_->parent);
    if (edgeHasStepTaken(parent_, current_, RIGHT_FOOTSTEP)){
      std::cout << " -- gen. left footstep neighbors --" << std::endl;
      generateFootstepNeighbors(LEFT_FOOTSTEP);
    }else if (edgeHasStepTaken(parent_, current_, LEFT_FOOTSTEP)){
      std::cout << " -- gen. right footstep neighbors --" << std::endl;
      generateFootstepNeighbors(RIGHT_FOOTSTEP);
    }else{
      std::cout << " -- gen. left footstep neighbors --" << std::endl;
      generateFootstepNeighbors(LEFT_FOOTSTEP);      
      std::cout << " -- gen. right footstep neighbors --" << std::endl;
      generateFootstepNeighbors(RIGHT_FOOTSTEP);
    }
    std::cout << "num neighbors generated: " << neighbors.size() << std::endl;


    return neighbors;
  }

  // Print the node path
  void LocomanipulationPlanner::printPath(){
  }



  // Edge identification between the current node and its parent
  bool LocomanipulationPlanner::edgeHasStepTaken(shared_ptr<LMVertex> from_node, shared_ptr<LMVertex> to_node, int footstep_side){
    double epsilon = 1e-6; 
    double pos_difference, theta_difference;

    if (footstep_side == RIGHT_FOOTSTEP){
      pos_difference = (to_node->right_foot.position - from_node->right_foot.position).norm();
      theta_difference = fabs(getAngle(to_node->right_foot.orientation) - getAngle(from_node->right_foot.orientation));
    }else{
      pos_difference = (to_node->left_foot.position - from_node->left_foot.position).norm();
      theta_difference = fabs(getAngle(to_node->left_foot.orientation) - getAngle(from_node->left_foot.orientation));
    }

    // std::cout << "footstep = " << (footstep_side == RIGHT_FOOTSTEP ? "right step": "left step") << std::endl;
    // std::cout << "pos difference = " << pos_difference << std::endl;
    // std::cout << "theta difference = " << theta_difference << std::endl;

    if ((pos_difference <= epsilon) && (theta_difference <= epsilon)) {
      // std::cout << "no footstep has been taken" << std::endl;
      return false;
    }else{  
      // std::cout << "footstep has been taken" << std::endl;
      return true;          
    }

  }

  bool LocomanipulationPlanner::edgeHasSVarMoved(shared_ptr<LMVertex> from_node, shared_ptr<LMVertex> to_node){
    double epsilon = 1e-6;
    double delta_s = fabs(to_node->s - from_node->s);

    if (delta_s <= epsilon){
       return false;
    }else{
       return true;
    }
  }

  // Helpers for setting up classifier input
  void LocomanipulationPlanner::addToXVector(const Eigen::Vector3d & pos, const Eigen::Quaterniond & ori, std::vector<double> & x){
    //Eigen::AngleAxisd tmp_aa(ori.normalized()); // gets the normalized version of ori and sets it to an angle axis representation
    tmp_aa = ori.normalized(); // gets the normalized version of ori and sets it to an angle axis representation
    tmp_ori_vec3 = tmp_aa.axis()*tmp_aa.angle();

    for(int i = 0; i < pos.size(); i++){
      x.push_back(pos[i]);
    }
    for(int i = 0; i < tmp_ori_vec3.size(); i++){
      x.push_back(tmp_ori_vec3[i]);
    }
  }

  void LocomanipulationPlanner::populateXVector(std::vector<double> & x, 
    const double & stance_origin_in, const double & manipulation_type_in,
    const Eigen::Vector3d & swing_foot_start_pos_in, const Eigen::Quaterniond & swing_foot_start_ori_in,  
    const Eigen::Vector3d & pelvis_pos_in, const Eigen::Quaterniond & pelvis_ori_in,  
    const Eigen::Vector3d & landing_foot_pos_in, const Eigen::Quaterniond & landing_foot_ori_in,  
    const Eigen::Vector3d & right_hand_start_pos_in, const Eigen::Quaterniond & right_hand_start_ori_in,  
    const Eigen::Vector3d & left_hand_start_pos_in, const Eigen::Quaterniond & left_hand_start_ori_in){

    x.push_back(stance_origin_in);
    x.push_back(manipulation_type_in);
    addToXVector(swing_foot_start_pos_in, swing_foot_start_ori_in, x);
    addToXVector(pelvis_pos_in, pelvis_ori_in, x);
    addToXVector(landing_foot_pos_in, landing_foot_ori_in, x);
    addToXVector(right_hand_start_pos_in, right_hand_start_ori_in, x);
    addToXVector(left_hand_start_pos_in, left_hand_start_ori_in, x);
  }

  Eigen::Vector3d LocomanipulationPlanner::quatToVec(const Eigen::Quaterniond & ori){
    Eigen::AngleAxisd tmp_ori(ori.normalized()); // gets the normalized version of ori and sets it to an angle axis representation
    Eigen::Vector3d ori_vec = tmp_ori.axis()*tmp_ori.angle();
    return ori_vec;
  }


  void LocomanipulationPlanner::normalizeInputCalculate(const Eigen::VectorXd & x_in, const Eigen::VectorXd & data_mean, const Eigen::VectorXd & data_std_dev, Eigen::VectorXd & x_normalized) {
    x_normalized = (x_in - data_mean).cwiseQuotient(data_std_dev);
  }


  double LocomanipulationPlanner::getClassifierResult(){
    // Using CPP neural network
    if (enable_cpp_nn){
      // std::cout << "constructing input" << std::endl;
      // Construct the input to the neural network
      // Stance and Manipulation Type
      cpp_nn_rawDatum[0] = nn_stance_origin;
      cpp_nn_rawDatum[1] = nn_manipulation_type;
      // Swing Foot Pos, Ori
      cpp_nn_rawDatum[2] = nn_swing_foot_start_pos[0];
      cpp_nn_rawDatum[3] = nn_swing_foot_start_pos[1];
      cpp_nn_rawDatum[4] = nn_swing_foot_start_pos[2];
      tmp_ori_vec3 = quatToVec(nn_swing_foot_start_ori);
      cpp_nn_rawDatum[5] = tmp_ori_vec3[0];
      cpp_nn_rawDatum[6] = tmp_ori_vec3[1];
      cpp_nn_rawDatum[7] = tmp_ori_vec3[2];
      // Pelvis Pos, Ori
      cpp_nn_rawDatum[8] = nn_pelvis_pos[0];
      cpp_nn_rawDatum[9] = nn_pelvis_pos[1];
      cpp_nn_rawDatum[10] = nn_pelvis_pos[2];
      tmp_ori_vec3 = quatToVec(nn_pelvis_ori);
      cpp_nn_rawDatum[11] = tmp_ori_vec3[0];
      cpp_nn_rawDatum[12] = tmp_ori_vec3[1];
      cpp_nn_rawDatum[13] = tmp_ori_vec3[2];
      // Landing Foot Pos, Ori
      cpp_nn_rawDatum[14] = nn_landing_foot_pos[0];
      cpp_nn_rawDatum[15] = nn_landing_foot_pos[1];
      cpp_nn_rawDatum[16] = nn_landing_foot_pos[2];
      tmp_ori_vec3 = quatToVec(nn_landing_foot_ori);
      cpp_nn_rawDatum[17] = tmp_ori_vec3[0];
      cpp_nn_rawDatum[18] = tmp_ori_vec3[1];
      cpp_nn_rawDatum[19] = tmp_ori_vec3[2];
      // Right Hand Pos Ori
      cpp_nn_rawDatum[20] = nn_right_hand_start_pos[0];
      cpp_nn_rawDatum[21] = nn_right_hand_start_pos[1];
      cpp_nn_rawDatum[22] = nn_right_hand_start_pos[2];
      tmp_ori_vec3 = quatToVec(nn_right_hand_start_ori);
      cpp_nn_rawDatum[23] = tmp_ori_vec3[0];
      cpp_nn_rawDatum[24] = tmp_ori_vec3[1];
      cpp_nn_rawDatum[25] = tmp_ori_vec3[2];
      // Left Hand Pos Ori
      cpp_nn_rawDatum[26] = nn_left_hand_start_pos[0];
      cpp_nn_rawDatum[27] = nn_left_hand_start_pos[1];
      cpp_nn_rawDatum[28] = nn_left_hand_start_pos[2];
      tmp_ori_vec3 = quatToVec(nn_left_hand_start_ori);
      cpp_nn_rawDatum[29] = tmp_ori_vec3[0];
      cpp_nn_rawDatum[30] = tmp_ori_vec3[1];
      cpp_nn_rawDatum[31] = tmp_ori_vec3[2];
  
      // std::cout << "normalizing input input" << std::endl;
      // Normalize the input  
      normalizeInputCalculate(cpp_nn_rawDatum, nn_mean, nn_std, cpp_nn_normalized_datum);

      // std::cout << "placing to a row" << std::endl;      
      // Put normalized datum as part of the neural net input matrix
      for (int i = 0; i < cpp_nn_input_size; i++){
        cpp_nn_data_input.row(i) = cpp_nn_normalized_datum;
      }

      cpp_nn_pred = nn_model->GetOutput(cpp_nn_data_input);
      // Get the first result only
      prediction_result = cpp_nn_pred(0,0);
      // std::cout << "Prediction: " << prediction_result << std::endl;
      return prediction_result;

    }else{
      // Otherwise we must be using the ROS client

      // Prepare classifier input
      classifier_srv.request.x.clear();
      classifier_srv.request.x.reserve(classifier_input_dim);
      populateXVector(classifier_srv.request.x, nn_stance_origin, nn_manipulation_type,
                                                nn_swing_foot_start_pos, nn_swing_foot_start_ori,
                                                nn_pelvis_pos, nn_pelvis_ori,
                                                nn_landing_foot_pos, nn_landing_foot_ori,
                                                nn_right_hand_start_pos, nn_right_hand_start_ori,
                                                nn_left_hand_start_pos, nn_left_hand_start_ori);
      // Reset prediction result to a negative value
      prediction_result = -1.0;
      
      while (true){
        // Try to call the classifier
        try {  
          // Call the classifier client
          if (classifier_client.call(classifier_srv)){
            prediction_result = classifier_srv.response.y;
            break;
            // ROS_INFO("Prediction: %0.4f", srv.response.y);
          }else{
              ROS_ERROR("Failed to call service locomanipulation_feasibility_classifier");
          }
        }
        catch(...){
        // Catch all exceptions
          std::cout << "Error exception occurred when calling service locomanipulation_feasibility_classifier" << std::endl;
        }

      }


      return prediction_result;
    }

  }

  void LocomanipulationPlanner::storeTransitionDatawithTaskSpaceInfo(const shared_ptr<LMVertex> & start_node_traj, bool result){
    // Add transition data
    std::cout << "storing transition data..." << std::endl;
    // Define the save path
    std::string userhome = std::string("/home/") + std::string(std::getenv("USER")) + std::string("/");
    std::cout << "User home path: " << userhome << std::endl;
    std::string parent_folder_path("Data/planner_data/"); 


    // Define the yaml emitter
    YAML::Emitter out;
    
    // Begin map creation
    out << YAML::BeginMap;
    data_saver::emit_string(out, "result", (result ? "success" : "failure"));
    data_saver::emit_joint_configuration(out, "q_init", start_node_traj->q_init);

    // Stance origin
    str_stance_origin = "";
    str_manipulation_type = "";
    if (nn_stance_origin == CONTACT_TRANSITION_DATA_LEFT_FOOT_STANCE){
      str_stance_origin = "left_foot";
    }
    else if (nn_stance_origin == CONTACT_TRANSITION_DATA_RIGHT_FOOT_STANCE){
      str_stance_origin = "right_foot";
    }

    // Manipulation Type
    if (nn_manipulation_type == CONTACT_TRANSITION_DATA_RIGHT_HAND){
      str_manipulation_type = "right_hand";
    }
    else if (nn_manipulation_type == CONTACT_TRANSITION_DATA_LEFT_HAND){
      str_manipulation_type = "left_hand";
    }
    else if (nn_manipulation_type == CONTACT_TRANSITION_DATA_BOTH_HANDS){
      str_manipulation_type = "both_hands"; 
    }


    // // generate HASH instead of integer counter
    std::size_t hash_number = getDataHash();
    std::string result_folder = result ? "positive_examples/" : "negative_examples/";
    std::string save_path = userhome + parent_folder_path + str_manipulation_type + "/transitions_data_with_task_space_info/" + result_folder + str_manipulation_type + "_" + str_stance_origin + "_" + std::to_string(hash_number) + ".yaml";
    std::cout << "saving to: " << save_path << std::endl;

    data_saver::emit_string(out, "stance_origin", str_stance_origin);
    data_saver::emit_position(out, "swing_foot_starting_position", nn_swing_foot_start_pos);
    data_saver::emit_orientation(out, "swing_foot_starting_orientation", nn_swing_foot_start_ori);
    data_saver::emit_position(out, "pelvis_starting_position", nn_pelvis_pos);
    data_saver::emit_orientation(out, "pelvis_starting_orientation", nn_pelvis_ori);

    data_saver::emit_string(out, "manipulation_type", str_manipulation_type);
    data_saver::emit_position(out, "left_hand_starting_position", nn_left_hand_start_pos);
    data_saver::emit_orientation(out, "left_hand_starting_orientation", nn_left_hand_start_ori);
    data_saver::emit_position(out, "right_hand_starting_position", nn_right_hand_start_pos);
    data_saver::emit_orientation(out, "right_hand_starting_orientation", nn_right_hand_start_ori);

    data_saver::emit_position(out, "landing_foot_position", nn_landing_foot_pos);
    data_saver::emit_orientation(out, "landing_foot_orientation", nn_landing_foot_ori);
    out << YAML::EndMap;

    // Store the data
    std::ofstream file_output_stream(save_path);     
    // std::cout << out.c_str() << std::endl;
    file_output_stream << out.c_str(); 
  }


  void LocomanipulationPlanner::appendPosString(const Eigen::Vector3d & pos, std::string & str_in_out){
    double factor = 1000.0;  
    str_in_out = str_in_out + std::to_string((int)round(pos[0] * factor)) + "_" + 
                              std::to_string((int)round(pos[1] * factor)) + "_" + 
                              std::to_string((int)round(pos[2] * factor));
  }
  void LocomanipulationPlanner::appendOriString(const Eigen::Quaterniond & ori, std::string & str_in_out){
    double factor = 1000.0;  
    str_in_out = str_in_out + std::to_string((int)round(ori.x() * factor)) + "_" + 
                              std::to_string((int)round(ori.y() * factor)) + "_" +
                              std::to_string((int)round(ori.z() * factor)) + "_" +
                              std::to_string((int)round(ori.w() * factor));                               
  }  
  std::size_t LocomanipulationPlanner::getDataHash(){
    std::string hash_string;
    // Begin creating hash string for this data
    hash_string = str_stance_origin + str_manipulation_type;
    appendPosString(nn_swing_foot_start_pos, hash_string); appendOriString(nn_swing_foot_start_ori, hash_string);
    appendPosString(nn_pelvis_pos, hash_string); appendOriString(nn_pelvis_ori, hash_string);
    appendPosString(nn_left_hand_start_pos, hash_string); appendOriString(nn_left_hand_start_ori, hash_string);
    appendPosString(nn_right_hand_start_pos, hash_string); appendOriString(nn_right_hand_start_ori, hash_string);
    appendPosString(nn_landing_foot_pos, hash_string); appendOriString(nn_landing_foot_ori, hash_string);

    std::cout << "hash string = " << std::endl;
    std::cout << hash_string << std::endl;

    return std::hash<std::string>{}(hash_string);
  }


}

