#include <avatar_locomanipulation/walking/walking_pattern_generator.hpp>

WalkingPatternGenerator::WalkingPatternGenerator(){
    std::cout << "[WalkingPatternGenerator] Constructed" << std::endl;
}

WalkingPatternGenerator::~WalkingPatternGenerator(){

}

void WalkingPatternGenerator::initialize_footsteps_rvrp(const std::vector<Footstep> & input_footstep_list, 
                                                        const Footstep & initial_footstance,
                                                        bool clear_list){
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
  rvrp_list.push_back(initial_rvrp); 
  dcm_ini_list.push_back(initial_rvrp);   
  dcm_eos_list.push_back(initial_rvrp);   
  initialize_footsteps_rvrp(input_footstep_list, initial_footstance);
}
