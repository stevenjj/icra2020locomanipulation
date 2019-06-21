#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>

int const WalkingPatternGenerator::SWING_VRP_TYPE = 0;
int const WalkingPatternGenerator::DOUBLE_SUPPORT_TRANSFER_VRP_TYPE = 1;

WalkingPatternGenerator::WalkingPatternGenerator(){
    std::cout << "[WalkingPatternGenerator] Constructed" << std::endl;
} 

WalkingPatternGenerator::~WalkingPatternGenerator(){

}

void WalkingPatternGenerator::initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, 
                                                        const Footstep & initial_footstance,
                                                        bool clear_list){
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

void WalkingPatternGenerator::get_average_rvrp(const Footstep & footstance_1, const Footstep & footstance_2, Eigen::Vector3d & average_rvrp){
  Eigen::Vector3d desired_rvrp(0, 0, z_vrp); // From foot local frame
  average_rvrp = 0.5*((footstance_1.R_ori*desired_rvrp + footstance_1.position) + (footstance_2.R_ori*desired_rvrp + footstance_2.position));
}

Eigen::Vector3d WalkingPatternGenerator::computeDCM_ini_i(const Eigen::Vector3d & r_vrp_d_i, const double & t_step, const Eigen::Vector3d & dcm_eos_i){
  return r_vrp_d_i + std::exp(-t_step/b)*(dcm_eos_i - r_vrp_d_i);
}

void WalkingPatternGenerator::computeDCM_states(){ 
  std::cout << "size of rvrp list = " << rvrp_list.size() << std::endl;

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
    return t_it;
  }else if (rvrp_type_list[step_i] == SWING_VRP_TYPE){
    return t_ds + t_ss;
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

double WalkingPatternGenerator::get_total_trajectory_time(){
  double total_time = 0.0;
  for(int i = 0; i < rvrp_type_list.size(); i++){
    total_time += get_t_step(i);
  }
  return total_time;
}

  // Swing trajectory calculation
void WalkingPatternGenerator::computeSE3_trajectory(const Footstep & init_location, const Footstep & landing_location){
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

  // Populate SE3 trajectory object

}

void WalkingPatternGenerator::initialize_trajectory_discretization(const int & N_samples){
  N_size = N_samples;
  double t_trajectory = get_total_trajectory_time() + t_settle;
  double dt = t_trajectory/N_size;

  traj_SE3_tmp = TrajSE3(N_size, dt);
  traj_SE3_left_foot = TrajSE3(N_size, dt);
  traj_SE3_right_foot = TrajSE3(N_size, dt);
  traj_ori_pelvis = TrajOrientation(N_size, dt);
  traj_pos_com = TrajEuclidean(3, N_size, dt);

}

void WalkingPatternGenerator::construct_trajectories(){
  double time = 0.0;
  double t_trajectory = get_total_trajectory_time() + t_settle;
  double dt = t_trajectory/N_size;

  traj_SE3_tmp.set_dt(dt);
  traj_SE3_left_foot.set_dt(dt);
  traj_SE3_right_foot.set_dt(dt);
  traj_ori_pelvis.set_dt(dt);
  traj_pos_com.set_dt(dt);  

}














