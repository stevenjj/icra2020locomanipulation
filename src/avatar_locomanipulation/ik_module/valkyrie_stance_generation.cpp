#include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>


ValkyrieStanceGeneration::ValkyrieStanceGeneration(){
	default_initialization();	
}

ValkyrieStanceGeneration::ValkyrieStanceGeneration(std::shared_ptr<RobotModel> & robot_model_in){
	default_initialization();
	this->setRobotModel(robot_model_in);
}

// Destructors
ValkyrieStanceGeneration::~ValkyrieStanceGeneration(){	
}

void ValkyrieStanceGeneration::setRobotModel(std::shared_ptr<RobotModel> & robot_model_in){
	robot_model = robot_model_in;
  // Set Posture Task names
  initializeTasks();
}

void ValkyrieStanceGeneration::initializeTasks(){
  // Create Task Stack
  std::vector< std::shared_ptr<Task> > priority_1_task_stack = {pelvis_wrt_mf_task, lfoot_contact_normal_task, rfoot_contact_normal_task, torso_neck_arm_posture_task};
  std::vector< std::shared_ptr<Task> > priority_2_task_stack = {lfoot_wrt_rfoot_task, overall_posture_task};

  // Set Pelvis Tasks
  pelvis_wrt_mf_task.reset(new Task6DPosewrtMidFeet(robot_model, "pelvis"));

  // Set Foot Tasks
  lfoot_contact_normal_task.reset(new TaskContactNormalTask(robot_model, "leftCOP_Frame", left_floor_normal, left_floor_center));
  rfoot_contact_normal_task.reset(new TaskContactNormalTask(robot_model, "rightCOP_Frame", right_floor_normal, right_floor_center));
  lfoot_wrt_rfoot_task.reset(new Task6DPosewrtFrame(robot_model, "leftCOP_Frame", "rightCOP_Frame"));

  rpalm_task.reset(new Task6DPose(robot_model, "rightPalm"));
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
  }else{
    // otherwise add a right palm 6d pose task
    priority_1_task_stack.push_back(rpalm_task);    
  }

  // If not using the left hand, add left arm joints to the posture task
  if (!(use_left_hand)){
    for(int i = 0; i < left_arm_joint_names.size(); i++){
      torso_neck_arm_posture_task_names.push_back(left_arm_joint_names[i]);
    }  
  }else{
    // otherwise add a left palm 6d pose task
    priority_1_task_stack.push_back(lpalm_task);    
  }

  // If both hands are not being used simultaneously, add torso joint tasks.
  if (!(use_left_hand && use_right_hand)){
    for(int i = 0; i < left_arm_joint_names.size(); i++){
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
  task_stack_priority_1.reset(new TaskStack(robot_model, priority_1_task_stack));
  task_stack_priority_2.reset(new TaskStack(robot_model, priority_2_task_stack));   

}


void ValkyrieStanceGeneration::default_initialization(){
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

  left_arm_joint_names = {"leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch"};
  right_arm_joint_names = {"rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch"};
  torso_joint_names = {"torsoYaw", "torsoPitch", "torsoRoll"};
  neck_joint_names = {"lowerNeckPitch", "neckYaw", "upperNeckPitch"};

  // torso_neck_arm_posture_task_names = {"torsoYaw", "torsoPitch", "torsoRoll", 
  //                                            "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", 
  //                                            "lowerNeckPitch", "neckYaw", "upperNeckPitch"};

}
