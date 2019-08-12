#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>

FeasibilityDataGenerator::FeasibilityDataGenerator(){
	common_initialization();
}

FeasibilityDataGenerator::~FeasibilityDataGenerator(){
}

void FeasibilityDataGenerator::common_initialization(){
	ik_start_config_module.reset(new IKModule());
	ctg.reset(new ConfigTrajectoryGenerator());

	// Initialize training case
	data_gen_manipulation_case = CASE_MANIPULATION_RIGHT_HAND;
  data_gen_stance_case = CASE_STANCE_LEFT_FOOT;

	// Initialize task references
  left_footstep.setLeftSide();
  right_footstep.setRightSide();

	pelvis_pos.setZero();
	pelvis_ori.setIdentity();	

  // Initialize temporary variables
  stance_foot_pos.setZero();
  stance_foot_ori.setIdentity();

  swing_foot_pos.setZero();
  swing_foot_ori.setIdentity();
  swing_foot_theta_angle = 0.0;

  // Reserve foot contact list sizes
  int contact_size = left_footstep.global_contact_point_list.size() + right_footstep.global_contact_point_list.size();
  foot_contact_list_3d.reserve(contact_size);
  foot_contact_list_2d.reserve(contact_size);
  // Populate with zeroes
  for(int i = 0; i < contact_size; i++){
    foot_contact_list_3d.push_back(Eigen::Vector3d::Zero(3));
    foot_contact_list_2d.push_back(math_utils::Point(0.0, 0.0));
  }


}

void FeasibilityDataGenerator::setRobotModel(std::shared_ptr<RobotModel> & robot_model_in){
	robot_model = robot_model_in;
  initializeConfigurationLimits();
	ik_start_config_module->setRobotModel(robot_model_in);
  initializeStartingIKTasks();
	ctg->setRobotModel(robot_model_in);
	ctg->commonInitialization();
}

void FeasibilityDataGenerator::setStartingIKConfig(const Eigen::VectorXd & q_ik_start_in){
  q_ik_start = q_ik_start_in;
  ik_start_config_module->setInitialConfig(q_ik_start);
}


void FeasibilityDataGenerator::initializeSeed(unsigned int seed_number){
	srand(seed_number);
  generator.seed(seed_number);
}

void FeasibilityDataGenerator::initializeStartingIKTasks(){
	// Initialize IK tasks
	upper_body_joint_names.clear();
	upper_body_joint_names = {"torsoYaw", "torsoPitch", "torsoRoll", 
		                        "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch",
                            "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch", 
                            "lowerNeckPitch", "neckYaw", "upperNeckPitch"};
  joint_pos = Eigen::VectorXd::Zero(upper_body_joint_names.size());

	upper_body_config_task.reset(new TaskJointConfig(robot_model, upper_body_joint_names));
	left_foot_task.reset(new Task6DPose(robot_model, "leftCOP_Frame"));
	right_foot_task.reset(new Task6DPose(robot_model, "rightCOP_Frame") );
	pelvis_task.reset(new Task6DPose(robot_model, "pelvis"));	

	// Create Task Stack Vector
	vec_task_stack.clear();
	vec_task_stack = {upper_body_config_task, left_foot_task, right_foot_task, pelvis_task};
	ik_task_stack.reset(new TaskStack(robot_model, vec_task_stack));

	// Clear the task hierarchy in the IK module
	ik_start_config_module->clearTaskHierarchy();
	// Add the tasks to the task hierarchy in the ik module
	ik_start_config_module->addTasktoHierarchy(ik_task_stack);
	// Prepare the ik module data structure
	ik_start_config_module->prepareNewIKDataStrcutures();
}

void FeasibilityDataGenerator::initializeConfigurationLimits(){
  q_min = robot_model->q_lower_pos_limit;
  q_max = robot_model->q_upper_pos_limit;

  q_min.head(7) = -10 * Eigen::VectorXd::Ones(7);
  q_max.head(7) = 10 * Eigen::VectorXd::Ones(7);
}

double FeasibilityDataGenerator::generateRandMinMax(const double min, const double max){
  return (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *(max-min) + min;
 }

bool FeasibilityDataGenerator::randomizeStartingConfiguration(){
  // Set stance foot to be the origin
  stance_foot_pos.setZero();
  stance_foot_ori.setIdentity();

  // Check if the sign needs to be flipped.
  double direction = 1.0;
  if (data_gen_stance_case == CASE_STANCE_LEFT_FOOT){
    direction = -1.0;
  }

  // Set Randomize desired swing foot starting pose
  swing_foot_pos[0] = generateRandMinMax(min_reach, max_reach); // x
  swing_foot_pos[1] = direction*generateRandMinMax(min_width, max_width); // y
  swing_foot_pos[2] = 0.0; // z

  // Set swing foot angle theta
  swing_foot_theta_angle = direction*generateRandMinMax(min_theta, max_theta); // theta
  swing_foot_ori = Eigen::AngleAxisd(swing_foot_theta_angle, Eigen::Vector3d(0.0, 0.0, 1.0) );

  // Set Left foot to be the stance otherwise, set right foot to be the stance for all other cases
  if (data_gen_stance_case == CASE_STANCE_LEFT_FOOT){
    left_footstep.setPosOri(stance_foot_pos, stance_foot_ori);
    right_footstep.setPosOri(swing_foot_pos, swing_foot_ori);
  }else{
    right_footstep.setPosOri(stance_foot_pos, stance_foot_ori);
    left_footstep.setPosOri(swing_foot_pos, swing_foot_ori);
  }

  // Set pelvis orientation to be the midfoot frame orientation
  pelvis_ori = Eigen::AngleAxisd( swing_foot_theta_angle/2.0, Eigen::Vector3d(0.0, 0.0, 1.0) );
  // Randomize pelvis location based on swing and stance feet location
  getRandomPelvisLocation(pelvis_pos);

  // randomize joint configuration
  q_rand = (pinocchio::randomConfiguration(robot_model->model, q_min, q_max));
  // Set Neck to 0.0
  q_rand[robot_model->getJointIndex("lowerNeckPitch")] = 0.0;
  q_rand[robot_model->getJointIndex("neckYaw")] = 0.0;
  q_rand[robot_model->getJointIndex("upperNeckPitch")] = 0.0;  
   for(int i = 0; i < upper_body_joint_names.size(); i++){
    joint_pos[i] = q_rand[robot_model->getJointIndex(upper_body_joint_names[i])];
    // also set the starting ik configuration to the randomized upper body joint configuration:
    q_ik_start[robot_model->getJointIndex(upper_body_joint_names[i])] = joint_pos[i];
  }

  // Set the starting configuration for the upper body joints:
  setStartingIKConfig(q_ik_start);

  // Set IK references
  upper_body_config_task->setReference(joint_pos);
  left_foot_task->setReference(left_footstep.position, left_footstep.orientation);
  right_foot_task->setReference(right_footstep.position, right_footstep.orientation);
  pelvis_task->setReference(pelvis_pos, pelvis_ori);

  // Prepare IK output
  int solve_result;
  double total_error_norm;
  std::vector<double> task_error_norms;
  Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(robot_model->getDimQdot()); 
  int ik_verbosity_level = IK_VERBOSITY_LOW;
  bool config_convergence = false;

  // Set Verbosity Level
  ik_start_config_module->setVerbosityLevel(ik_verbosity_level);
  // Solve IK
  config_convergence = ik_start_config_module->solveIK(solve_result, task_error_norms, total_error_norm, q_sol);
  // ik_start_config_module->printSolutionResults();

  // Check if CoM height position is within limits
  bool com_height_within_limits = false;
  if  ((com_height_min <= robot_model->x_com[2]) && (robot_model->x_com[2] <= com_height_max)){
    com_height_within_limits = true;
  }else{
    return false;
  }

  // If we found a starting configuration set it to be the starting config.
  if (config_convergence){
    q_start = q_sol;
  }

  return (config_convergence && com_height_within_limits);
}

void FeasibilityDataGenerator::getRandomPelvisLocation(Eigen::Vector3d & pelvis_out){
  // get Possible Hull Vertices (based on stance foot and swing foot poses)
  getFeetVertexList();   
  // compute Hull Vertices
  contact_hull_vertices = math_utils::convexHull(foot_contact_list_2d); 
  // randomly select a line segment from the hull vertices
  std::uniform_int_distribution<int> u_distribution_int(1, contact_hull_vertices.size()-1);
  int segment_head_idx = u_distribution_int(generator);
  int segment_tail_idx = segment_head_idx - 1;

  // head and tail vectors which defines the line segment.
  Eigen::Vector3d segment_head(contact_hull_vertices[segment_head_idx].x, contact_hull_vertices[segment_head_idx].y, 0.0);
  Eigen::Vector3d segment_tail(contact_hull_vertices[segment_tail_idx].x, contact_hull_vertices[segment_tail_idx].y, 0.0);

  // randomly select a point on the line segment
  std::uniform_real_distribution<double> u_distribution_double(0.0, 1.0);
  double alpha = u_distribution_double(generator);
  Eigen::Vector3d point_on_segment = alpha*segment_head + (1.0-alpha)*segment_tail;

  // randomly select x,y point from midfeet to point on the line segment.
  Eigen::Vector3d midfeet_position =  0.5*(stance_foot_pos + swing_foot_pos);
  alpha = u_distribution_double(generator)*convex_hull_percentage;
  Eigen::Vector3d rand_pelvis_pos = alpha*point_on_segment + (1.0-alpha)*midfeet_position;
  // ^-- this is the pelvis x,y position

  // Randomize pelvis height
  rand_pelvis_pos[2] = generateRandMinMax(pelvis_height_min, pelvis_height_max); // set pelvis height

  // Set the output
  pelvis_out = rand_pelvis_pos;
}

void FeasibilityDataGenerator::getFeetVertexList(){
  // Get Vertices of the left foot contact points
  for(int i = 0; i < left_footstep.global_contact_point_list.size(); i++){
    foot_contact_list_3d[i][0] = left_footstep.global_contact_point_list[i][0];
    foot_contact_list_3d[i][1] = left_footstep.global_contact_point_list[i][1];
    foot_contact_list_3d[i][2] = left_footstep.global_contact_point_list[i][2];
  }

  // Get Vertices of the right foot contact points  
  int offset = left_footstep.global_contact_point_list.size();
  for(int i = 0; i < right_footstep.global_contact_point_list.size(); i++){
    foot_contact_list_3d[i + offset][0] = right_footstep.global_contact_point_list[i][0];
    foot_contact_list_3d[i + offset][1] = right_footstep.global_contact_point_list[i][1];
    foot_contact_list_3d[i + offset][2] = right_footstep.global_contact_point_list[i][2];
  }

  // Get the x,y locations only
  for(int i = 0; i < foot_contact_list_3d.size(); i++){
    foot_contact_list_2d[i].x = foot_contact_list_3d[i][0];
    foot_contact_list_2d[i].y = foot_contact_list_3d[i][1];
  }

}

void FeasibilityDataGenerator::randomizeFootLandingConfiguration(){
  // Check which foot is the swing foot
  double direction = 1.0;
  if (data_gen_stance_case == CASE_STANCE_LEFT_FOOT){
    direction = -1.0;
    // If stance foot is left, then swing foot is the right side
    landing_footstep.setRightSide();
  }else{
    // stance foot is right, so swing foot is the left side
    landing_footstep.setLeftSide();    
  }

  // Set Randomize desired swing foot landing pose
  swing_foot_pos[0] = generateRandMinMax(min_reach, max_reach); // x
  swing_foot_pos[1] = direction*generateRandMinMax(min_width, max_width); // y
  swing_foot_pos[2] = 0.0; // z

  // Set swing foot angle theta
  swing_foot_theta_angle = direction*generateRandMinMax(min_theta, max_theta); // theta
  swing_foot_ori = Eigen::AngleAxisd(swing_foot_theta_angle, Eigen::Vector3d(0.0, 0.0, 1.0) );

  // Set the landing foot location
  landing_footstep.setPosOri(swing_foot_pos, swing_foot_ori);
  landing_footstep.printInfo();
}


void FeasibilityDataGenerator::initializeConfigTrajectoryGenerationModule(){
  // Prepare the config trajectory generator
  // Set resolution of the config trajectory generator
  ctg->initializeDiscretization(N_resolution);

  // Don't restrict the torso joints
  ctg->setUseTorsoJointPosition(false);
  // Have fast double support times
  ctg->wpg.setDoubleSupportTime(0.2);

  // Check if the right hand is being used
  if  ((data_gen_manipulation_case == CASE_MANIPULATION_RIGHT_HAND) || 
      (data_gen_manipulation_case == CASE_MANIPULATION_BOTH_HANDS)){
    ctg->setUseRightHand(true);
  }

  // Check if the left hand is being used
  if  ((data_gen_manipulation_case == CASE_MANIPULATION_LEFT_HAND) || 
      (data_gen_manipulation_case == CASE_MANIPULATION_BOTH_HANDS)){
    ctg->setUseLeftHand(true);
  }

  // Reinitialize the task stack
  ctg->reinitializeTaskStack();
}

// Randomly generate a contact transition data
bool FeasibilityDataGenerator::generateContactTransitionData(){
  bool start_configuration = false;

  // Generate a random starting configuration
  while (start_configuration != true){
    start_configuration = randomizeStartingConfiguration();    
  }
  // Randomize a foot landing configuration
  randomizeFootLandingConfiguration();    

  // Set the hand configuration
  // Check if the right hand is being used
  if  ((data_gen_manipulation_case == CASE_MANIPULATION_RIGHT_HAND) || 
      (data_gen_manipulation_case == CASE_MANIPULATION_BOTH_HANDS)){
    // robot model would already have been updated after an initial configuration has been set
    Eigen::Vector3d rhand_pos;
    Eigen::Quaterniond rhand_ori;
    robot_model->getFrameWorldPose("rightPalm", rhand_pos, rhand_ori);  
    // Set Constant Right Hand trajectory to current
    ctg->setConstantRightHandTrajectory(rhand_pos, rhand_ori);
  }

  // Check if the left hand is being used
  if  ((data_gen_manipulation_case == CASE_MANIPULATION_LEFT_HAND) || 
      (data_gen_manipulation_case == CASE_MANIPULATION_BOTH_HANDS)){
    // robot model would already have been updated after an initial configuration has been set
    Eigen::Vector3d lhand_pos;
    Eigen::Quaterniond lhand_ori;
    robot_model->getFrameWorldPose("leftPalm", lhand_pos, lhand_ori);  
    // Set Constant Right Hand trajectory to current
    ctg->setConstantLeftHandTrajectory(lhand_pos, lhand_ori);
  }  

  // Set the landing footstep to try
  std::vector<Footstep> input_footstep_list = {landing_footstep};

  // Try to compute the trajectory
  bool trajectory_convergence = ctg->computeConfigurationTrajectory(q_start, input_footstep_list);

  // Return the trajectory convergence result
  return trajectory_convergence;

}

/*
initializeSeed();
initializeStartingIKTasks();



loadStartingConfiguration();
initializeConfigurationLimits();


checkCOMinHull();

// use case for each of
randomizeConfiguration();
	generateRandXYFromHullPoints

setStartingIKTasks();

randomizeDesiredFinalConfiguration();
setConfigurationTrajectoryTasks();


storeInitConfig();
storeNegativeExamplePair()
storePositiveExamplePair();
storeTrajectoryConfig();


//
generateData();


generateData(){
	desired_positive_examples = 1000;
	count_positive_examples = 0;

	// set which hand to use for ctg


	while (count_positive_examples < desired_positive_examples){
		randomizeConfiguration();
		bool initial_configuration_convergence = solveIK();
		
		// visualizeInitial()

		if !(initial_configuration_convergence){
			continue; // Repeat the randomization
		}
		storeInitConfig();

		randomizeDesiredFinalConfiguration();

		// Set starting CoM position,Pelvis Ori, Footstance
		bool trajectory_convergence = ctg->computeTrajectory();

		// visualizeTrajectory()

		if !(trajectory_convergence){
			storeNegativeExamplePair()
			continue; // Repeat the randomization
		}
		storePositiveExamplePair();
		storeTrajectoryConfig();
	}





	

	
}


*/