#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>

FeasibilityDataGenerator::FeasibilityDataGenerator(){
	initialize_modules();
}

FeasibilityDataGenerator::~FeasibilityDataGenerator(){
}

void FeasibilityDataGenerator::initialize_modules(){
	ik_start_config_module.reset(new IKModule());
	ctg.reset(new ConfigTrajectoryGenerator());
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

/*
initializeStartingIKTasks();



initializeSeed();
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