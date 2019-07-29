#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>

int const WalkingPatternGenerator::SWING_VRP_TYPE = 0;
int const WalkingPatternGenerator::DOUBLE_SUPPORT_TRANSFER_VRP_TYPE = 1;

int const WalkingPatternGenerator::STATE_SWING = 0;
int const WalkingPatternGenerator::STATE_DOUBLE_SUPPORT = 1;
int const WalkingPatternGenerator::STATE_FINAL_TRANSFER = 2;

WalkingPatternGenerator::WalkingPatternGenerator(){
    std::cout << "[WalkingPatternGenerator] Constructed" << std::endl;
} 

WalkingPatternGenerator::~WalkingPatternGenerator(){

}


// Sets the desired CoM Height
void WalkingPatternGenerator::setCoMHeight(double z_vrp_in){
  z_vrp = z_vrp_in;
} 

// Sets the desired double support time
void WalkingPatternGenerator::setDoubleSupportTime(double t_ds_in){
  t_ds = t_ds_in;
}

// Sets the desired single support swing time
void WalkingPatternGenerator::setSingleSupportSwingTime(double t_ss_in){
  t_ss = t_ss_in;
} 

// Percentage to settle. Default 0.999
void WalkingPatternGenerator::setSettlingPercentage(double percentage_convergence){
  t_settle = -b*log(1 - percentage_convergence);
} 

// Sets the swing height of the robot. Default 0.1m
void WalkingPatternGenerator::setSwingHeight(double swing_height_in){
  swing_height = swing_height_in;
}

void WalkingPatternGenerator::initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, 
                                                        const Footstep & initial_footstance,
                                                        bool clear_list){
  // Store the input footstep list
  footstep_list = input_footstep_list;
  if (input_footstep_list.size() == 0){ 
    return;
  }
  // clear DCM variables if true
  if (clear_list){
    rvrp_list.clear();
    dcm_ini_list.clear();
    dcm_eos_list.clear();    
  }

  // Create an rvrp for the stance leg
  Eigen::Vector3d current_rvrp(0, 0, z_vrp); // From foot local frame
  Eigen::Vector3d current_stance_rvrp(0, 0, z_vrp); // The stance leg from the foot local frame
  Eigen::Vector3d left_stance_rvrp(0, 0, z_vrp); // a left stance leg from the foot local frame
  Eigen::Vector3d right_stance_rvrp(0, 0, z_vrp); // a right stance leg from the foot local frame

  current_stance_rvrp = initial_footstance.R_ori * current_stance_rvrp + initial_footstance.position;
  left_stance_rvrp = current_stance_rvrp;
  right_stance_rvrp = current_stance_rvrp;

  // Specify that this is the eos for the previous rvrp
  rvrp_type_list.push_back(DOUBLE_SUPPORT_TRANSFER_VRP_TYPE);
  dcm_eos_list.push_back(current_stance_rvrp);

  // Add an rvrp to transfer to the stance leg
  rvrp_list.push_back(current_stance_rvrp);
  dcm_ini_list.push_back(current_stance_rvrp);

  int previous_step = initial_footstance.robot_side;

  for(int i = 0; i < input_footstep_list.size(); i++){
    // initialize rvrp to [0, 0, z_vrp] in terms of the foot's local frame
    current_rvrp.setZero(); current_rvrp[2] = z_vrp;
    // Get the world frame representation
    current_rvrp = input_footstep_list[i].R_ori * current_rvrp + input_footstep_list[i].position;

    // Set the correct stance rvrp
    current_stance_rvrp = input_footstep_list[i].robot_side == LEFT_FOOTSTEP ? right_stance_rvrp : left_stance_rvrp;
    // If this is the last step, use the average rvrp between the stance and current rvrp
    if (i == (input_footstep_list.size() - 1)){
      current_rvrp = 0.5*(current_rvrp + current_stance_rvrp);
    }

    // ----------- Begin handling same robot side footsteps -------------
    // If taking a footstep on the same side, first go to the stance foot
    if (input_footstep_list[i].robot_side == previous_step){
      // Specify that this is the eos for the previous rvrp
      rvrp_type_list.push_back(DOUBLE_SUPPORT_TRANSFER_VRP_TYPE);
      dcm_eos_list.push_back(current_stance_rvrp);
      // Add a new rvrp
      rvrp_list.push_back(current_stance_rvrp);
      dcm_ini_list.push_back(current_stance_rvrp);   
    }
    else{
      // otherwise, update the correct stance to the latest rvrp
      if (input_footstep_list[i].robot_side == LEFT_FOOTSTEP){
        left_stance_rvrp = current_rvrp;
      }else{
        right_stance_rvrp = current_rvrp;
      }
    }
    // -----------------------------------------------------------------
    // Specify that this is the eos for the previous rvrp
    rvrp_type_list.push_back(SWING_VRP_TYPE);
    dcm_eos_list.push_back(current_rvrp);   

    // Add this rvrp to the list and also populate the DCM states
    rvrp_list.push_back(current_rvrp);
    dcm_ini_list.push_back(current_rvrp);   

    // Update previous_step side 
    previous_step = input_footstep_list[i].robot_side;
  }
}

void WalkingPatternGenerator::initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, const Footstep & initial_footstance, const Eigen::Vector3d & initial_rvrp){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  rvrp_list.clear();
  dcm_ini_list.clear();
  dcm_eos_list.clear();

  // Add the initial virtual repellant point. 
  rvrp_list.push_back(initial_rvrp); 
  dcm_ini_list.push_back(initial_rvrp);   
  // Add the remaining virtual repellant points   
  initialize_footsteps_rvrp(input_footstep_list, initial_footstance);
}

void WalkingPatternGenerator::initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, const Footstep & left_footstance, const Footstep & right_footstance){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  Eigen::Vector3d initial_rvrp; 
  get_average_rvrp(left_footstance, right_footstance, initial_rvrp);
  // Set the stance leg
  if (input_footstep_list[0].robot_side == LEFT_FOOTSTEP){
    initialize_footsteps_rvrp(input_footstep_list, right_footstance, initial_rvrp);
  }else{
    initialize_footsteps_rvrp(input_footstep_list, left_footstance, initial_rvrp);
  }

}

void WalkingPatternGenerator::initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, const Footstep & left_footstance, const Footstep & right_footstance, const Eigen::Vector3d & initial_com){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  // Set the stance leg
  if (input_footstep_list[0].robot_side == LEFT_FOOTSTEP){
    initialize_footsteps_rvrp(input_footstep_list, right_footstance, initial_com);
  }else{
    initialize_footsteps_rvrp(input_footstep_list, left_footstance, initial_com);
  }

}


void WalkingPatternGenerator::get_average_rvrp(const Footstep & footstance_1, const Footstep & footstance_2, Eigen::Vector3d & average_rvrp){
  Eigen::Vector3d desired_rvrp(0, 0, z_vrp); // From foot local frame
  average_rvrp = 0.5*((footstance_1.R_ori*desired_rvrp + footstance_1.position) + (footstance_2.R_ori*desired_rvrp + footstance_2.position));
}

Eigen::Vector3d WalkingPatternGenerator::computeDCM_ini_i(const Eigen::Vector3d & r_vrp_d_i, const double & t_step, const Eigen::Vector3d & dcm_eos_i){
  return r_vrp_d_i + std::exp(-t_step/b)*(dcm_eos_i - r_vrp_d_i);
}

void WalkingPatternGenerator::computeDCM_states(){ 
  // std::cout << "size of rvrp list = " << rvrp_list.size() << std::endl;

  // Use backwards recursion to compute the initial and final dcm states
  double t_step = 0.0;
  for (int i = dcm_ini_list.size()-2; i >= 0; i--){
    // Get the t_step to use for backwards integration
    t_step = get_t_step(i);
    // Compute dcm_ini for step i
    dcm_ini_list[i] = computeDCM_ini_i(rvrp_list[i], t_step, dcm_eos_list[i]);

    // Set dcm_eos for step i-1
    if (i > 0){
      dcm_eos_list[i-1] = dcm_ini_list[i];
    }

  }

}

void WalkingPatternGenerator::initialize_internal_clocks(){
  internal_timer = 0.0;
  internal_step_timer = 0.0;
  internal_step_i = 0;
  internal_t_step = get_t_step(internal_step_i);
}

double WalkingPatternGenerator::get_t_step(const int & step_i){
  // Use transfer time for double support and overall step time for swing types
  if ((rvrp_type_list[step_i]) == DOUBLE_SUPPORT_TRANSFER_VRP_TYPE){
    return t_ds;
  }else if (rvrp_type_list[step_i] == SWING_VRP_TYPE){
    if (step_i == (rvrp_type_list.size() - 1)){
      return t_ss; // If this is the last step, use the swing time
    }else{
      return t_ds + t_ss; // Otherwise perform a double support transfer as well
    }
  }else{
    return t_ds + t_ss;
  }
}

Eigen::Vector3d WalkingPatternGenerator::get_next_desired_DCM(const double & dt){
  // Increment internal step timer so long as it's below the t_step
  if (internal_step_timer < internal_t_step){
    internal_timer += dt;
    internal_step_timer += dt;        
  }
  // Reset timer if it exceeds the t_step and there are more virtual repellant points to process
  if ((internal_step_timer >= internal_t_step) && (internal_step_i < rvrp_type_list.size()-1)){
    internal_step_timer = 0.0;
    internal_step_i += 1;
    internal_t_step = get_t_step(internal_step_i);
  }
  // Clamp internal step timer if it's bigger
  if (internal_step_timer > internal_t_step){
    internal_step_timer = internal_t_step;
  }

  return rvrp_list[internal_step_i] + std::exp( (internal_step_timer-internal_t_step) / b)*(dcm_eos_list[internal_step_i] - rvrp_list[internal_step_i]);
}

Eigen::Vector3d WalkingPatternGenerator::get_desired_DCM(const int & step_index, const double & t){
  // Get t_step
  double t_step = get_t_step(step_index);
  double time = t;
  // Clamp time value
  if (t < 0){
    time = 0;
  }
  else if (t > t_step){
    time = t_step;
  }
  return rvrp_list[step_index] + std::exp( (time-t_step) / b)*(dcm_eos_list[step_index] - rvrp_list[step_index]);
}

Eigen::Vector3d WalkingPatternGenerator::get_com_vel(const Eigen::Vector3d & current_com, const int & step_index, const double & t){
  return (-1.0/b)*(current_com - get_desired_DCM(step_index, t));
}


double WalkingPatternGenerator::get_total_trajectory_time(){
  double total_time = 0.0;
  for(int i = 0; i < rvrp_type_list.size(); i++){
    total_time += get_t_step(i);
  }
  return total_time + t_settle;
}

void WalkingPatternGenerator::initialize_trajectory_discretization(const int & N_samples){
  N_size = N_samples;
  double dt = 0.1;   // set dummy dt

  traj_SE3_tmp = TrajSE3(N_size, dt);
  traj_SE3_left_foot = TrajSE3(N_size, dt);
  traj_SE3_right_foot = TrajSE3(N_size, dt);
  traj_ori_pelvis = TrajOrientation(N_size, dt);
  traj_pos_com = TrajEuclidean(3, N_size, dt);
  traj_dcm_pos = TrajEuclidean(3, N_size, dt);

}




void WalkingPatternGenerator::construct_trajectories(const std::vector<Footstep> & input_footstep_list, 
                                                     const Footstep & initial_left_footstance,
                                                     const Footstep & initial_right_footstance, 
                                                     const Eigen::Vector3d & initial_com,
                                                     const Eigen::Quaterniond initial_pelvis_ori){
  
  // Initialize r_VRP and r_vrp types
  initialize_footsteps_rvrp(input_footstep_list, initial_left_footstance, initial_right_footstance, initial_com);
  // Compute DCM boundary conditions
  computeDCM_states();

  // Set internal dt
  internal_dt = get_total_trajectory_time()/N_size; 
  double dt = internal_dt;
  traj_SE3_tmp.set_dt(dt);
  traj_SE3_left_foot.set_dt(dt);
  traj_SE3_right_foot.set_dt(dt);
  traj_ori_pelvis.set_dt(dt);
  traj_pos_com.set_dt(dt);  
  traj_dcm_pos.set_dt(dt);

  // Construct State and Bin Size Lists
  compute_trajectory_lists();

  // Debug
  // for(int i = 0; i < state_list.size(); i++){
  //   std::cout << "State " << i << ": " << state_list[i] << std::endl;
  //   std::cout << "  Bins:" << bin_size_list[i] << std::endl;
  // }

  // Begin constructing trajectories
  compute_com_dcm_trajectory(initial_com);
  compute_pelvis_orientation_trajectory(initial_pelvis_ori, initial_left_footstance, initial_right_footstance);
  compute_foot_trajectories(initial_left_footstance, initial_right_footstance);
}


void WalkingPatternGenerator::compute_trajectory_lists(){
  // Compute the state and bin size lists
  double t_trajectory = get_total_trajectory_time();
  double dt = t_trajectory/N_size;
  int running_bin_size = 0;

  // Setting bin sizes for swing and double support trajectories
  int N_swing = int(t_ss/dt);
  int N_DS = int(t_ds/dt);

  // std::cout << "t_trajectory = " << t_trajectory << std::endl;
  // std::cout << "dt = " << dt << std::endl;
  // std::cout << "N_size = " << N_size << std::endl;
  // std::cout << "N_swing = " << N_swing << std::endl;
  // std::cout << "N_DS = " << N_DS << std::endl;

  for(int i = 0; i < rvrp_type_list.size(); i++){
    // If it's a swing, there is always a double support phase except for the last step
    if (rvrp_type_list[i] == SWING_VRP_TYPE){
      // Use single support time in swing phase
      state_list.push_back(STATE_SWING);
      bin_size_list.push_back(N_swing);
      running_bin_size += N_swing;
      // If this is the last step, don't add a double support phase
      if (i == (rvrp_type_list.size() - 1)){
        continue;
      }
      // Use double support time in double support phase
      state_list.push_back(STATE_DOUBLE_SUPPORT);
      bin_size_list.push_back(N_DS);
      running_bin_size += N_DS;
    }
    else if (rvrp_type_list[i] == DOUBLE_SUPPORT_TRANSFER_VRP_TYPE){
      state_list.push_back(STATE_DOUBLE_SUPPORT);
      bin_size_list.push_back( N_DS );
      running_bin_size += N_DS;
    }
  }
  // Always append a final transfer at the end of the rvrp list
  state_list.push_back(STATE_FINAL_TRANSFER);
  bin_size_list.push_back(N_size - running_bin_size);
}

void WalkingPatternGenerator::setOrientationTrajectory(const int & starting_index, const int & N_bins, HermiteQuaternionCurve & curve, TrajOrientation & traj_ori){
  double s = 0.0;
  Eigen::Quaterniond quat;
  for(size_t i = 0; i < N_bins; i++){
    s = (double) i/N_bins;
    curve.evaluate(s, quat);
    traj_ori.set_quat(starting_index + i, quat);
  }

}

void WalkingPatternGenerator::compute_com_dcm_trajectory(const Eigen::Vector3d & initial_com){
  Eigen::Vector3d com_pos = initial_com;
  Eigen::Vector3d dcm_pos; dcm_pos.setZero();
  int step_index = 0;
  double t = 0.0;
  double t_step = get_t_step(step_index);
  double t_prev = 0.0;
  double dt = internal_dt;

  // Always try to compute CoM finely. Also, ensure that we follow the desired discretization
  double dt_local = 1e-3; // Use this discretization for integrating the CoM
  int N_local = int(get_total_trajectory_time()/dt_local);
  int j = 0;

  // In case N_size is larger than N_local: 
  if (N_size > N_local){
    // Compute using N_size as the discretization. This is rarely the case
    for(int i = 0; i < N_size; i++){
      // x_post = dx*dt + x_pre
      t = dt*i;
      com_pos = get_com_vel(com_pos, step_index, t-t_prev)*dt + com_pos;
      dcm_pos = get_desired_DCM(step_index, t-t_prev);
      // Check if t-t_prev exceeded the current t_step and if we can increment the step index
      if ( ((t-t_prev) >= t_step) && (step_index < rvrp_type_list.size()-1) ){
        step_index++;
        t_prev = t;
        t_step = get_t_step(step_index);        
      }

        // std::cout << com_pos.transpose() << std::endl;
        // Store the CoM position
        traj_pos_com.set_pos(i, com_pos);
        traj_dcm_pos.set_pos(i, dcm_pos);  
    }
  }else{
    // Compute with a more fine integration of the CoM. This is usually the case
    for(int i = 0; i < N_local; i++){
      if (j < N_size){
        // x_post = dx*dt + x_pre
        t = dt_local*i;
        com_pos = get_com_vel(com_pos, step_index, t-t_prev)*dt_local + com_pos;
        dcm_pos = get_desired_DCM(step_index, t-t_prev);
        // Check if t-t_prev exceeded the current t_step and if we can increment the step index
        if ( ((t-t_prev) >= t_step) && (step_index < rvrp_type_list.size()-1) ){
          step_index++;
          t_prev = t;
          t_step = get_t_step(step_index);        
        }

        // Store the COM position at the desired discretization
        if (i % (N_local/N_size) == 0){
          // std::cout << com_pos.transpose() << std::endl;
          traj_pos_com.set_pos(j, com_pos);
          traj_dcm_pos.set_pos(j, dcm_pos);
          j++;      
        }
      }
      else{
        break;
      }
    }
  }


}

void WalkingPatternGenerator::compute_pelvis_orientation_trajectory(const Eigen::Quaterniond & init_pelvis_ori,
                                                         const Footstep & initial_left_footstance,
                                                         const Footstep & initial_right_footstance){
  Footstep prev_left_stance = initial_left_footstance;
  Footstep prev_right_stance = initial_right_footstance;

  Footstep stance_step;
  Footstep target_step;
  Footstep midfeet;

  Eigen::Quaterniond current_pelvis_ori = init_pelvis_ori;

  int trajectory_index = 0;
  int step_counter = 0;

  // std::cout << "Pelvis Ori Length = " << traj_ori_pelvis.get_trajectory_length() << std::endl;

  // Go through each state and compute pelvis orientation trajectory
  for(int state_index = 0; state_index < state_list.size(); state_index++){
    // Swing transfers should go to the midframe.
    if (state_list[state_index] == STATE_SWING){
      // Compute the orientation waypoint
      target_step = footstep_list[step_counter];
      // Get the stance location and update the stance location 
      if (target_step.robot_side == LEFT_FOOTSTEP){
        stance_step = prev_right_stance;
        prev_left_stance = target_step;
      }else{
        stance_step = prev_left_stance;
        prev_right_stance = target_step;
      }
      // Midfeet orientation is the waypoint orientation
      midfeet.computeMidfeet(stance_step, target_step, midfeet);
      // index-1 is valid because we never start with the swing state.
      traj_ori_pelvis.get_quat(trajectory_index-1, current_pelvis_ori);
      // Set quaternion curve with 0 ang vel and ang acc boundary conditions
      HermiteQuaternionCurve quat_curve(current_pelvis_ori, Eigen::Vector3d(0,0,0), midfeet.orientation, Eigen::Vector3d(0,0,0));
      // Set the orientation trajectory     
      setOrientationTrajectory(trajectory_index, bin_size_list[state_index], quat_curve, traj_ori_pelvis);
      trajectory_index += bin_size_list[state_index];
      // Update the new current_pelvis_orientation
      traj_ori_pelvis.get_quat(trajectory_index-1, current_pelvis_ori);
      // Consider the next swing step
      step_counter++;
    }
    // Initial transfers, double support, and final transfers should keep orientation constant
    else{
      for(int i = 0; i < bin_size_list[state_index]; i++){
        traj_ori_pelvis.set_quat(trajectory_index, current_pelvis_ori);
        trajectory_index++;
      }
    }
  } // Finish setting orientation trajectory

}



void WalkingPatternGenerator::compute_foot_trajectories(const Footstep & initial_left_footstance, const Footstep & initial_right_footstance){
  Footstep current_left_foot = initial_left_footstance;
  Footstep current_right_foot = initial_right_footstance;

  Footstep stance_step;
  Footstep target_step;
  int trajectory_index = 0;
  int step_counter = 0;

  // Go through each state and compute the foot trajectories
  for(int state_index = 0; state_index < state_list.size(); state_index++){
    // Swing foot has a trajectory during transfers, but stance foot is stationary.
    if (state_list[state_index] == STATE_SWING){
      // Get the target step.
      target_step = footstep_list[step_counter];
      // Compute the feet trajectories
      if (target_step.robot_side == LEFT_FOOTSTEP){
        // Compute swing trajectory for the left foot and set right foot to stationary
        setSwingFootTrajectory(current_left_foot, target_step, trajectory_index, bin_size_list[state_index], traj_SE3_left_foot);
        setConstantSE3(trajectory_index,  bin_size_list[state_index], traj_SE3_right_foot, current_right_foot.position, current_right_foot.orientation);
        // Update the current steps
        current_left_foot = target_step;
      }else{
        // Compute swing trajectory for the right foot and set left foot to stationary
        setSwingFootTrajectory(current_right_foot, target_step, trajectory_index, bin_size_list[state_index], traj_SE3_right_foot);
        setConstantSE3(trajectory_index,  bin_size_list[state_index], traj_SE3_left_foot, current_left_foot.position, current_left_foot.orientation);
        // Update the current steps
        current_right_foot = target_step;
      }
      // Update trajectory index
      trajectory_index += bin_size_list[state_index];
      // Consider the next footstep
      step_counter++;
    }
    // Feet are stationary during transfers 
    else{
      for(int i = 0; i < bin_size_list[state_index]; i++){
        traj_SE3_left_foot.set_pos(trajectory_index, current_left_foot.position, current_left_foot.orientation);
        traj_SE3_right_foot.set_pos(trajectory_index, current_right_foot.position, current_right_foot.orientation);
        trajectory_index++;
      }
    }

  }


}



void WalkingPatternGenerator::setSwingFootTrajectory(const Footstep & init_location, 
                                                     const Footstep & landing_location, 
                                                     const int & starting_index, 
                                                     const int & N_bins, 
                                                     TrajSE3 & swing_foot){

  // Compute where the foot will be in the middle of the trajectory
  mid_foot_.computeMidfeet(init_location, landing_location, mid_foot_);

  // Compute midfeet boundary conditions
  // Linear velocity at the middle of the swing is the total swing travel over swing time 
  Eigen::Vector3d mid_swing_local_foot_pos(0, 0, swing_height);
  Eigen::Vector3d mid_swing_position = mid_foot_.position + mid_foot_.R_ori*mid_swing_local_foot_pos;
  Eigen::Vector3d mid_swing_velocity = (landing_location.position - init_location.position)/t_ss;

  // Construct Position trajectory  
  HermiteCurveVec trajectory_init_to_mid(init_location.position, Eigen::Vector3d::Zero(3), 
                                         mid_swing_position, mid_swing_velocity);

  HermiteCurveVec trajectory_mid_to_end(mid_swing_position, mid_swing_velocity, 
                                        landing_location.position, Eigen::Vector3d::Zero(3));

  // Construct Quaternion trajectory
  Eigen::Vector3d ang_vel_start; ang_vel_start.setZero();
  Eigen::Vector3d ang_vel_end; ang_vel_end.setZero();
  HermiteQuaternionCurve foot_ori_trajectory(init_location.orientation, ang_vel_start,
                                             landing_location.orientation, ang_vel_end);

  // Populate SE3 trajectory object --------------
  int N_pre_mid = N_bins/2;
  double s = 0.0;
  Eigen::Quaterniond quat;
  Eigen::Vector3d pos;
  // Set the foot orientation trajectory
  for(size_t i = 0; i < N_bins; i++){
    s = (double) i/N_bins;
    foot_ori_trajectory.evaluate(s, quat);
    swing_foot.traj_ori.set_quat(starting_index + i, quat);
  }  
  // Set the position trajectory before the midpoint
  for(size_t i = 0; i < N_pre_mid; i++){
    s = (double) i/N_pre_mid;
    pos = trajectory_init_to_mid.evaluate(s);
    swing_foot.traj_pos.set_pos(starting_index + i, pos);
  }  
  // Set the position trajectory after the midpoint:
  for(size_t i = N_pre_mid; i < N_bins; i++){
    s = (double) (i-N_pre_mid)/(N_bins - N_pre_mid);
    pos = trajectory_mid_to_end.evaluate(s);
    swing_foot.traj_pos.set_pos(starting_index + i, pos);
  }    

}

void WalkingPatternGenerator::setConstantSE3(const int & starting_index, const int & N_bins, TrajSE3 & traj, const Eigen::Vector3d & pos, const Eigen::Quaterniond & quat){
  for(size_t i = 0; i < N_bins; i++){
    traj.set_pos(starting_index + i, pos, quat);
  }
}
