#include <iostream>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>

#include <avatar_locomanipulation/tasks/task.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>

void initialize_config(Eigen::VectorXd & q_init){
  std::cout << "Initialize Valkyrie Model" << std::endl;
  ValkyrieModel valkyrie;

  // X dimensional state vectors
  Eigen::VectorXd q_start;

  // Set origin at 0.0
  q_start = Eigen::VectorXd::Zero(valkyrie.getDimQ());
  double theta = 0.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q

  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[valkyrie.getJointIndex("leftHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;

  q_start[valkyrie.getJointIndex("rightShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie.getJointIndex("rightElbowPitch")] = 0.4;
  q_start[valkyrie.getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}


void testIK_module(){
  std::cout << "[IK Module Test]" << std::endl;
  Eigen::VectorXd q_init;
  initialize_config(q_init);

  // Create IK Module
  IKModule ik_module;
  ik_module.setInitialConfig(q_init);  

  // Create Tasks
  std::shared_ptr<Task> lfoot_task(new Task6DPose(ik_module.robot_model, "leftCOP_Frame"));
  std::shared_ptr<Task> rfoot_task(new Task6DPose(ik_module.robot_model, "rightCOP_Frame"));
  std::shared_ptr<Task> rpalm_task(new Task6DPose(ik_module.robot_model, "rightPalm"));

  // Stack Tasks in order of priority
  std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {lfoot_task, rfoot_task}));
  std::shared_ptr<Task> task_stack_priority_2(new TaskStack(ik_module.robot_model, {rpalm_task}));


  // Quick Test set Reference and get Error  
  ik_module.robot_model->updateFullKinematics(q_init);

  Eigen::VectorXd rp_pos_ref = Eigen::VectorXd::Zero(3);
  Eigen::Quaterniond rp_quat_ref; rp_quat_ref.setIdentity();
  Eigen::VectorXd rp_task_error; 
  rpalm_task->setReference(rp_pos_ref, rp_quat_ref);
  rpalm_task->getError(rp_task_error);
  std::cout << "Right Palm Task Error" << rp_task_error.transpose() << std::endl;

}

int main(int argc, char ** argv){
  testIK_module();
  return 0;
}