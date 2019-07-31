#include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>


ValkyrieStanceGeneration::ValkyrieStanceGeneration(){
	default_initialization();	
}

ValkyrieStanceGeneration::ValkyrieStanceGeneration(std::shared_ptr<RobotModel> & robot_model_in):robot_model(robot_model_in){
	default_initialization();
	this->setRobotModel(robot_model_in);
}

// Destructors
ValkyrieStanceGeneration::~ValkyrieStanceGeneration(){	
}

void ValkyrieStanceGeneration::setRobotModel(std::shared_ptr<RobotModel> & robot_model_in){
	robot_model = robot_model_in;
  snap_to_floor_ik_module.setRobotModel(robot_model_in);
  stance_ik_module.setRobotModel(robot_model_in);
}

void ValkyrieStanceGeneration::setStartingConfig(const Eigen::VectorXd & q_start_in){
  q_start = q_start_in;
  snap_to_floor_ik_module.setInitialConfig(q_start);
  stance_ik_module.setInitialConfig(q_start);
}

// Whether or not there is a desired hand pose
void ValkyrieStanceGeneration::setUseRightHand(bool use_right_hand_in){
  use_right_hand = use_right_hand_in;
}
void ValkyrieStanceGeneration::setUseLeftHand(bool use_left_hand_in){
  use_left_hand = use_left_hand_in;
}

// Set Desired Hand Poses
void ValkyrieStanceGeneration::setDesiredRightHandPose(const Eigen::Vector3d des_pos, const Eigen::Quaterniond des_quat){
  rpalm_des_pos = des_pos;
  rpalm_des_quat = des_quat;
}
void ValkyrieStanceGeneration::setDesiredLeftHandPose(const Eigen::Vector3d des_pos, const Eigen::Quaterniond des_quat){  
  lpalm_des_pos = des_pos;
  lpalm_des_quat = des_quat;
}

void ValkyrieStanceGeneration::initializeTasks(){
  // Set Base Task
  base_pose_task.reset(new Task6DPose(robot_model, "pelvis"));
  base_pose_task->setTaskGain(1e-1);

  // Set Pelvis Tasks
  pelvis_wrt_rf_task.reset(new TaskXDPosewrtFrame(robot_model, {TASK_DIM_X, TASK_DIM_Y, TASK_DIM_Z}, "pelvis", "rightCOP_Frame"));
  pelvis_wrt_mf_task.reset(new Task6DPosewrtMidFeet(robot_model, "pelvis"));

  // Set Foot Tasks
  lfoot_contact_normal_task.reset(new TaskContactNormalTask(robot_model, "leftCOP_Frame", left_floor_normal, left_floor_center));
  rfoot_contact_normal_task.reset(new TaskContactNormalTask(robot_model, "rightCOP_Frame", right_floor_normal, right_floor_center));
  lfoot_wrt_rfoot_task.reset(new TaskXDPosewrtFrame(robot_model, {TASK_DIM_X, TASK_DIM_Y, TASK_DIM_RZ}, "leftCOP_Frame", "rightCOP_Frame"));

  rpalm_task.reset(new Task6DPoseNoRXRY(robot_model, "rightPalm"));
  lpalm_task.reset(new Task6DPose(robot_model, "leftPalm"));

  // Clear Torso, neck, and arm Posture Task Names
  torso_neck_arm_posture_task_names.clear();
  // Add Neck to tasks as there is always a neck task
  for(int i = 0; i < neck_joint_names.size(); i++){
    torso_neck_arm_posture_task_names.push_back(neck_joint_names[i]);
  }

  // If not using the right hand, add right arm joints to the posture task
  if (!(use_right_hand)){
    for(int i = 0; i < right_arm_joint_names.size(); i++){
      torso_neck_arm_posture_task_names.push_back(right_arm_joint_names[i]);
    }  
  }

  // If not using the left hand, add left arm joints to the posture task
  if (!(use_left_hand)){
    for(int i = 0; i < left_arm_joint_names.size(); i++){
      torso_neck_arm_posture_task_names.push_back(left_arm_joint_names[i]);
    }  
  }

  // If neither hands are being used, add torso joint tasks.
  if (!(use_left_hand || use_right_hand)){
    for(int i = 0; i < torso_joint_names.size(); i++){
      torso_neck_arm_posture_task_names.push_back(torso_joint_names[i]);
    }  
  }

  // Set torso, neck, and arm posture tasks
  torso_neck_arm_posture_task.reset(new TaskJointConfig(robot_model, torso_neck_arm_posture_task_names));
  torso_neck_arm_posture_task->setTaskGain(1e-1);

  // Set overall posture task
  overall_posture_task_joint_names = robot_model->joint_names;
  overall_posture_task.reset(new TaskJointConfig(robot_model, overall_posture_task_joint_names));
  overall_posture_task->setTaskGain(1e-1);

  // Set the task stack
  createTaskStack();
}

void ValkyrieStanceGeneration::createTaskStack(){
  // Create Task Stack
  // std::vector< std::shared_ptr<Task> > priority_1_task_stack = {pelvis_wrt_mf_task, lfoot_contact_normal_task, rfoot_contact_normal_task, torso_neck_arm_posture_task};
  // std::vector< std::shared_ptr<Task> > priority_2_task_stack = {lfoot_wrt_rfoot_task, overall_posture_task, base_pose_task};

  // std::vector< std::shared_ptr<Task> > priority_1_task_stack = {lfoot_wrt_rfoot_task, pelvis_wrt_mf_task, torso_neck_arm_posture_task, lfoot_contact_normal_task, rfoot_contact_normal_task}; //{lfoot_wrt_rfoot_task, torso_neck_arm_posture_task, lfoot_contact_normal_task, rfoot_contact_normal_task};

  std::vector< std::shared_ptr<Task> > priority_1_task_stack = {torso_neck_arm_posture_task, pelvis_wrt_mf_task, lfoot_contact_normal_task, rfoot_contact_normal_task};
  std::vector< std::shared_ptr<Task> > priority_2_task_stack = {overall_posture_task, base_pose_task};

  // If the hand is being used, add it to the task stack
  if (use_right_hand){
    priority_1_task_stack.push_back(rpalm_task);    
  }

  if (use_left_hand){
    priority_1_task_stack.push_back(lpalm_task);    
  }

  std::vector< std::shared_ptr<Task> > snap_to_floor_tasks_priority_1 = {pelvis_wrt_mf_task, lfoot_wrt_rfoot_task, torso_neck_arm_posture_task, lfoot_contact_normal_task, rfoot_contact_normal_task};
  std::vector< std::shared_ptr<Task> > snap_to_floor_tasks_priority_2 = {overall_posture_task, base_pose_task};

  // Create the task stack
  task_stack_snap_to_floor_p1.reset(new TaskStack(robot_model, snap_to_floor_tasks_priority_1));
  task_stack_snap_to_floor_p2.reset(new TaskStack(robot_model, snap_to_floor_tasks_priority_2));

  task_stack_priority_1.reset(new TaskStack(robot_model, priority_1_task_stack));
  task_stack_priority_2.reset(new TaskStack(robot_model, priority_2_task_stack));   

  // Clear the task hierarchy and add the tasks to hierarchy
  snap_to_floor_ik_module.clearTaskHierarchy();
  snap_to_floor_ik_module.addTasktoHierarchy(task_stack_snap_to_floor_p1);
  snap_to_floor_ik_module.addTasktoHierarchy(task_stack_snap_to_floor_p2);

  stance_ik_module.clearTaskHierarchy();
  stance_ik_module.addTasktoHierarchy(task_stack_priority_1);
  // stance_ik_module.addTasktoHierarchy(task_stack_priority_2);

  // Prepare the IK data structures
  snap_to_floor_ik_module.prepareNewIKDataStrcutures();
  stance_ik_module.prepareNewIKDataStrcutures();

}




void ValkyrieStanceGeneration::default_initialization(){
  // Set desired base configuration
  base_des_pos.setZero(); base_des_quat.setIdentity();

  // Set desired Pelvis configuration wrt right foot
  pelvis_wrt_rfoot_des_pos.setZero(); pelvis_wrt_rfoot_des_quat.setIdentity();
  pelvis_wrt_rfoot_des_pos[1] = 0.125;
  pelvis_wrt_rfoot_des_pos[2] = 1.0;

  // Set Default Desired Pelvis Location w.r.t midfeet frame
	pelvis_wrt_mf_des_pos.setZero(); pelvis_wrt_mf_des_pos[2] = 1.0;
	pelvis_wrt_mf_des_quat.setIdentity();

  // Set Default Contact normals of the feet
	left_floor_normal.setZero(); left_floor_normal[2] = 1.0;
	left_floor_center.setZero();
	right_floor_normal.setZero(); right_floor_normal[2] = 1.0;
	right_floor_center.setZero();

  // Set Default Left Foot pose with respect to right feet
	lf_wrt_rf_des_pos.setZero(); 
	lf_wrt_rf_des_pos[1] = 0.25;
	lf_wrt_rf_des_quat.setIdentity();

  // Set Default right and left hand desired locations
  lpalm_des_pos.setZero();
  lpalm_des_quat.setIdentity();
  rpalm_des_pos.setZero();
  rpalm_des_quat.setIdentity();

  // Set Joint names
  left_arm_joint_names = {"leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch"};
  right_arm_joint_names = {"rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch"};
  torso_joint_names = {"torsoYaw", "torsoPitch", "torsoRoll"};
  neck_joint_names = {"lowerNeckPitch", "neckYaw", "upperNeckPitch"};

}


bool ValkyrieStanceGeneration::computeStance(Eigen::VectorXd & q_out){
  // Initialize the tasks
  initializeTasks();
  
  // Set Remaining task references
  // Set desired base pose to be close to the initial configuration
  base_pose_task->setReference(Eigen::Vector3d(q_start[0], q_start[1], q_start[2]), Eigen::Quaterniond(q_start[6], q_start[3], q_start[4], q_start[5]));
  pelvis_wrt_rf_task->setReference(pelvis_wrt_rfoot_des_pos, pelvis_wrt_rfoot_des_quat);
  pelvis_wrt_mf_task->setReference(pelvis_wrt_mf_des_pos, pelvis_wrt_mf_des_quat);
  lfoot_wrt_rfoot_task->setReference(lf_wrt_rf_des_pos, lf_wrt_rf_des_quat);
  rpalm_task->setReference(rpalm_des_pos, rpalm_des_quat);
  lpalm_task->setReference(lpalm_des_pos, lpalm_des_quat);

  // Set Posture references
  Eigen::VectorXd q_des;
  getSelectedPostureTaskReferences(torso_neck_arm_posture_task_names, q_start, q_des);
  torso_neck_arm_posture_task->setReference(q_des);

  getSelectedPostureTaskReferences(overall_posture_task_joint_names, q_start, q_des);
  overall_posture_task->setReference(q_des);

  // Set IK Descent parameters
  snap_to_floor_ik_module.setSequentialDescent(false);
  snap_to_floor_ik_module.setBackTrackwithCurrentTaskError(true);
  snap_to_floor_ik_module.setCheckPrevViolations(true);
  snap_to_floor_ik_module.setEnableInertiaWeighting(false);

  // stance_ik_module.setSequentialDescent(false);
  stance_ik_module.setSequentialDescent(true);
  stance_ik_module.setBackTrackwithCurrentTaskError(true);
  stance_ik_module.setCheckPrevViolations(true);
  stance_ik_module.setEnableInertiaWeighting(false);

  // Perform the sequential IK:
  bool snap_to_floor_task_convergence = false;
  bool stance_identification_task_convergence = false;

  // Step 1: snap the robot to the floor
  snap_to_floor_task_convergence = snap_to_floor_ik_module.solveIK(solve_result, task_error_norms, total_error_norm, q_sol);

  // Step 2: Solve for the end effector pose 
  if (snap_to_floor_task_convergence){
    this->setStartingConfig(q_sol);
    stance_identification_task_convergence = stance_ik_module.solveIK(solve_result, task_error_norms, total_error_norm, q_sol);
  }

  // Output the solution
  q_out = q_sol;

  return (snap_to_floor_task_convergence && stance_identification_task_convergence);

}


void ValkyrieStanceGeneration::getSelectedPostureTaskReferences(const std::vector<std::string> & selected_names, const Eigen::VectorXd & q_config, Eigen::VectorXd & q_ref){
  Eigen::VectorXd q_des;
  q_des = Eigen::VectorXd::Zero(selected_names.size());

  // Use the initial configuration to find the reference vector for the posture task
  for(int i = 0; i < selected_names.size(); i++){
    // std::cout << selected_names[i] << std::endl;
    q_des[i] = q_config[robot_model->getJointIndex(selected_names[i])];
  }
  q_ref = q_des;  
}