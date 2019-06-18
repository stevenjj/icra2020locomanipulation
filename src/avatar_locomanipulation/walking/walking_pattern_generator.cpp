#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>

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

  // Add an rvrp to the initial footstance
  Eigen::Vector3d current_rvrp(0, 0, z_vrp); // From foot local frame
  Eigen::Vector3d current_stance_rvrp(0, 0, z_vrp); // The stance leg from the foot local frame
  Eigen::Vector3d left_stance_rvrp(0, 0, z_vrp); // a left stance leg from the foot local frame
  Eigen::Vector3d right_stance_rvrp(0, 0, z_vrp); // a right stance leg from the foot local frame

  current_stance_rvrp = initial_footstance.R_ori * current_stance_rvrp + initial_footstance.position;
  left_stance_rvrp = current_stance_rvrp;
  right_stance_rvrp = current_stance_rvrp;

  // Add an rvrp to transfer to the stance leg
  rvrp_list.push_back(current_stance_rvrp);
  dcm_ini_list.push_back(current_stance_rvrp);
  dcm_eos_list.push_back(current_stance_rvrp);

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
      // This is a transfer type of footstep transition
      rvrp_list.push_back(current_stance_rvrp);
      dcm_ini_list.push_back(current_stance_rvrp);   
      dcm_eos_list.push_back(current_stance_rvrp);
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

    // Add this rvrp to the list and also populate the DCM states
    rvrp_list.push_back(current_rvrp);
    dcm_ini_list.push_back(current_rvrp);   
    dcm_eos_list.push_back(current_rvrp);   

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

  // Add the initial virtual repellant point. // This is a transfer type
  rvrp_list.push_back(initial_rvrp); 
  dcm_ini_list.push_back(initial_rvrp);   
  dcm_eos_list.push_back(initial_rvrp);
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
