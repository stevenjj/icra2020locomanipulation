#include <avatar_locomanipulation/feasibility/feasibility_data_playback.hpp>


FeasibilityDataPlayBack::FeasibilityDataPlayBack(){	
	common_initialization();
	std::cout << "[FeasibilityDataPlayBack] Initialized" << std::endl;
}

FeasibilityDataPlayBack::~FeasibilityDataPlayBack(){
}

void FeasibilityDataPlayBack::common_initialization(){
	ctg.reset(new ConfigTrajectoryGenerator());

  swing_foot_position.setZero();
  swing_foot_orientation.setIdentity();

	landing_foot_position.setZero();
	landing_foot_orientation.setIdentity();

	right_hand_position.setZero();
	right_hand_orientation.setIdentity();

	left_hand_position.setZero();
	left_hand_orientation.setIdentity();

}

void FeasibilityDataPlayBack::setRobotModel(std::shared_ptr<RobotModel> & robot_model_in){	
	robot_model = robot_model_in;
	ctg->setRobotModel(robot_model_in);
	ctg->commonInitialization();
}

void FeasibilityDataPlayBack::loadParamFile(const std::string filepath){
  param_handler.load_yaml_file(filepath);

  param_handler.getInteger("N_resolution", N_resolution);
  param_handler.getValue("walking_com_height", walking_com_height);
  param_handler.getValue("walking_double_support_time", walking_double_support_time);
  param_handler.getValue("walking_single_support_time", walking_single_support_time);
  param_handler.getValue("walking_settling_percentage", walking_settling_percentage);
  param_handler.getValue("walking_swing_height", walking_swing_height);

  ctg->initializeDiscretization(N_resolution); // Sets the discretization resolution
  ctg->wpg.setCoMHeight(walking_com_height); // Sets the desired CoM Height
  ctg->wpg.setDoubleSupportTime(walking_double_support_time); // Sets the desired double support / transfer time time
  ctg->wpg.setSingleSupportSwingTime(walking_single_support_time); // Sets the desired single support swing time
  ctg->wpg.setSettlingPercentage(walking_settling_percentage); // Percentage to settle. Default 0.999
  ctg->wpg.setSwingHeight(walking_swing_height); // Sets the swing height of the robot. Default 0.1m

  std::cout << "[FeasibilityDataPlayBack] Data Generation Parameters:" << std::endl;
  std::cout << "  N_resolution: " << N_resolution << std::endl;  
  std::cout << "  walking_com_height: " << walking_com_height << std::endl;  
  std::cout << "  walking_double_support_time: " << walking_double_support_time << std::endl;  
  std::cout << "  walking_single_support_time: " << walking_single_support_time << std::endl;  
  std::cout << "  walking_settling_percentage: " << walking_settling_percentage << std::endl;  
  std::cout << "  walking_swing_height: " << walking_swing_height << std::endl;  
}


void FeasibilityDataPlayBack::loadData(const std::string filepath){
  std::cout << "[FeasibilityDataPlayBack] Loading " << filepath << std::endl;
  // Load the YAML file
	param_handler.load_yaml_file(filepath);

	// Load the initial configuration
  getParamVec("q_init", q_start);

	// Update Robot Model
	robot_model->updateFullKinematics(q_start);

  // Load the manipulation type and stance origin
  param_handler.getString("stance_origin", stance_foot);
	param_handler.getString("manipulation_type", manipulation_type);

  // Load the starting swing foot configuration
  getParamPos("swing_foot_starting_position", swing_foot_position);
  getParamOri("swing_foot_starting_orientation", swing_foot_orientation);

  // Load the foot landing configuration
  getParamPos("landing_foot_position", landing_foot_position);
  getParamOri("landing_foot_orientation", landing_foot_orientation);


  // Set the landing footstep
  if (stance_foot.compare("left_foot") == 0){
    std::cout << "left stance foot" << std::endl;
    // If stance foot is left, then swing foot is the right side
    landing_footstep.setRightSide();
  }else{
    // stance foot is right, so swing foot is the left side
    std::cout << "right stance foot" << std::endl;
    landing_footstep.setLeftSide();    
  }
  landing_footstep.setPosOri(landing_foot_position, landing_foot_orientation);
  landing_footstep.printInfo();

  // Load the hand SE3 positions
  getParamPos("right_hand_starting_position", right_hand_position);
  getParamOri("right_hand_starting_orientation", right_hand_orientation);

  getParamPos("left_hand_starting_position", left_hand_position);
  getParamOri("left_hand_starting_orientation", left_hand_orientation);


  std::cout << "  q_init = " << q_start.transpose() << std::endl;
  std::cout << "  stance_origin = " << stance_foot << std::endl;
  std::cout << "  manipulation_type = " << manipulation_type << std::endl;

  std::cout << "  swing_foot_starting_position = " << swing_foot_position.transpose() << std::endl;
  std::cout << "  swing_foot_starting_orientation = "; math_utils::printQuat(swing_foot_orientation);

  std::cout << "  landing_foot_position = " << landing_foot_position.transpose() << std::endl;
  std::cout << "  landing_foot_orientation = "; math_utils::printQuat(landing_foot_orientation);

  std::cout << "  right_hand_starting_position = " << right_hand_position.transpose() << std::endl;
  std::cout << "  right_hand_starting_orientation = "; math_utils::printQuat(right_hand_orientation);

  std::cout << "  left_hand_starting_position = " << left_hand_position.transpose() << std::endl;
  std::cout << "  left_hand_starting_orientation = "; math_utils::printQuat(left_hand_orientation);

  // Compare hand SE3 positions with the stored q_init configuration
	if ((manipulation_type.compare("right_hand") == 0) || (manipulation_type.compare("both_hands") == 0)){
    ctg->setUseRightHand(true);
	  std::cout << "[FeasibilityDataPlayBack] Using the right hand" << std::endl;    

    Eigen::Vector3d rhand_pos;
    Eigen::Quaterniond rhand_ori;
    robot_model->getFrameWorldPose("rightPalm", rhand_pos, rhand_ori);  
    std::cout << "  robot rightPalm pos " << rhand_pos.transpose() << std::endl;
    std::cout << "  robot rightPalm ori "; math_utils::printQuat(rhand_ori);
	}
  if ((manipulation_type.compare("left_hand") == 0) || (manipulation_type.compare("both_hands") == 0)){
    ctg->setUseLeftHand(true);
    std::cout << "[FeasibilityDataPlayBack] Using the left hand" << std::endl;    

    Eigen::Vector3d lhand_pos;
    Eigen::Quaterniond lhand_ori;
    robot_model->getFrameWorldPose("leftPalm", lhand_pos, lhand_ori);  
    std::cout << "  robot leftPalm pos " << lhand_pos.transpose() << std::endl;
    std::cout << "  robot leftPalm ori "; math_utils::printQuat(lhand_ori);
  }


}

void FeasibilityDataPlayBack::getParamVec(const std::string param_name, Eigen::VectorXd & vec){
  // load values to temporary container
  std::vector<double> standard_vec;  
  param_handler.getVector(param_name, standard_vec);

  // Set eigen vector size and set values
  vec = Eigen::VectorXd::Zero(standard_vec.size());
  for(int i = 0; i < standard_vec.size(); i++){
    vec[i] = standard_vec[i];
  }

}

void FeasibilityDataPlayBack::getParamPos(const std::string param_name, Eigen::Vector3d & pos){
  param_handler.getNestedValue({param_name, "x"}, pos[0]);
  param_handler.getNestedValue({param_name, "y"}, pos[1]);
  param_handler.getNestedValue({param_name, "z"}, pos[2]);  
}

void FeasibilityDataPlayBack::getParamOri(const std::string param_name, Eigen::Quaterniond & ori){
  param_handler.getNestedValue({param_name, "x"}, ori.x());
  param_handler.getNestedValue({param_name, "y"}, ori.y());
  param_handler.getNestedValue({param_name, "z"}, ori.z());  
  param_handler.getNestedValue({param_name, "w"}, ori.w());      
}

bool FeasibilityDataPlayBack::playback(){
  ctg->setConstantRightHandTrajectory(right_hand_position, right_hand_orientation);
  ctg->setConstantLeftHandTrajectory(left_hand_position, left_hand_orientation);

  // Set the landing footstep to try
  std::vector<Footstep> input_footstep_list = {landing_footstep};

  // Try to compute the trajectory
  bool trajectory_convergence = ctg->computeConfigurationTrajectory(q_start, input_footstep_list);

  // Return the result
  return trajectory_convergence;
} 