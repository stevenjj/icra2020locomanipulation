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
	left_foot_pos.setZero();
	left_foot_ori.setIdentity();

	right_foot_pos.setZero();
	right_foot_ori.setIdentity();	

	pelvis_pos.setZero();
	pelvis_ori.setIdentity();	

  // Initialize temporary variables
  stance_foot_pos.setZero();
  stance_foot_ori.setIdentity();

  swing_foot_pos.setZero();
  swing_foot_ori.setIdentity();
  swing_foot_theta_angle = 0.0;
}

void FeasibilityDataGenerator::setRobotModel(std::shared_ptr<RobotModel> & robot_model_in){
	robot_model = robot_model_in;
	ik_start_config_module->setRobotModel(robot_model_in);
	ctg->setRobotModel(robot_model_in);
	ctg->commonInitialization();
}

void FeasibilityDataGenerator::initializeSeed(unsigned int seed_number){
	srand(seed_number);
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

  // Set pelvis pose to be the midfoot frame
  pelvis_ori = Eigen::AngleAxisd( swing_foot_theta_angle/2.0, Eigen::Vector3d(0.0, 0.0, 1.0) );

  // pelvis_pos[0] = generateRandMinMax(pelvis_height_min, pelvis_height_max);
  // pelvis_pos[1] = generateRandMinMax(pelvis_height_min, pelvis_height_max);
  pelvis_pos = 0.5*(stance_foot_pos + swing_foot_pos);
  pelvis_pos[2] = generateRandMinMax(pelvis_height_min, pelvis_height_max);
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