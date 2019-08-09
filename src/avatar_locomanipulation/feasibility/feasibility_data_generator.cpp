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

void FeasibilityDataGenerator::randomizeStartingConfiguration(){
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

  // Set pelvis pose to be the midfoot frame
  pelvis_ori = Eigen::AngleAxisd( swing_foot_theta_angle/2.0, Eigen::Vector3d(0.0, 0.0, 1.0) );

  // pelvis_pos[0] = generateRandMinMax(pelvis_height_min, pelvis_height_max);
  // pelvis_pos[1] = generateRandMinMax(pelvis_height_min, pelvis_height_max);

  getFeetVertexList();   // get Possible Hull Vertices (based on stance foot and swing foot poses)
  contact_hull_vertices = math_utils::convexHull(foot_contact_list_2d); // compute Hull Vertices
  std::cout << "contact hull vertices list size = " << contact_hull_vertices.size() << std::endl;
  for(int i = 0; i < contact_hull_vertices.size(); i++){
    std::cout << "contact_hull "<< i << " (x,y) = (" << contact_hull_vertices[i].x << ", " << contact_hull_vertices[i].y << ")" << std::endl;
  }
  
  std::uniform_int_distribution<int> distribution(1, contact_hull_vertices.size()-1);
  for(int i = 0 ; i < 100; i++){
    int rand_line_segment_index = distribution(generator);
    std::cout << "random index = " << rand_line_segment_index << std::endl;
  }


  // randomly select line segment from vertices
  // randomly select a point on the line segment
  // randomly select x,y point from midfeet to point on the line segment.
  // ^-- this is the pelvis x,y position


  pelvis_pos = 0.5*(stance_foot_pos + swing_foot_pos); // set pelvis x,y 
  pelvis_pos[2] = generateRandMinMax(pelvis_height_min, pelvis_height_max); // set pelvis height


  std::cout << "swing_foot_position " << swing_foot_pos.transpose() << std::endl;
  std::cout << "swing_foot_theta_angle " << swing_foot_theta_angle << std::endl;
  std::cout << "swing_foot_ori = "; math_utils::printQuat(swing_foot_ori);

  std::cout << "pelvis_pos " << pelvis_pos.transpose() << std::endl;
  std::cout << "pelvis_ori = "; math_utils::printQuat(pelvis_ori);

  // randomize joint configuration
  q_rand = (pinocchio::randomConfiguration(robot_model->model, q_min, q_max));
  // Set Neck to 0.0
  q_rand[robot_model->getJointIndex("lowerNeckPitch")] = 0.0;
  q_rand[robot_model->getJointIndex("neckYaw")] = 0.0;
  q_rand[robot_model->getJointIndex("upperNeckPitch")] = 0.0;  
   for(int i = 0; i < upper_body_joint_names.size(); i++){
    joint_pos[i] = q_rand[robot_model->getJointIndex(upper_body_joint_names[i])];
  }
  std::cout << "joint_pos = " << joint_pos.transpose() << std::endl;


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