#include <iostream>
#include <avatar_locomanipulation/ik_module/ik_module.hpp>

#include <avatar_locomanipulation/tasks/task.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>

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
  // Update Robot Kinematics
  ik_module.robot_model->updateFullKinematics(q_init);

  // Create Tasks
  std::shared_ptr<Task> lfoot_task(new Task6DPose(ik_module.robot_model, "leftCOP_Frame"));
  std::shared_ptr<Task> lfoot_ori_task(new Task3DOrientation(ik_module.robot_model, "leftCOP_Frame"));
  std::shared_ptr<Task> rfoot_task(new Task6DPose(ik_module.robot_model, "rightCOP_Frame"));

  std::shared_ptr<Task> com_task(new TaskCOM(ik_module.robot_model));

  std::shared_ptr<Task> rpalm_task(new Task6DPose(ik_module.robot_model, "rightPalm"));

  // Stack Tasks in order of priority
  // std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {lfoot_task, rfoot_task, rpalm_task}));
  std::shared_ptr<Task> task_stack_priority_1(new TaskStack(ik_module.robot_model, {lfoot_task, rfoot_task, com_task}));
  std::shared_ptr<Task> task_stack_priority_2(new TaskStack(ik_module.robot_model, {rpalm_task}));




   // Set desired Foot configuration
  Eigen::Vector3d rfoot_des_pos;
  Eigen::Quaternion<double> rfoot_des_quat;

  Eigen::Vector3d lfoot_des_pos;
  Eigen::Quaternion<double> lfoot_des_quat;

  // Foot should be flat on the ground and spaced out by 0.25m
  rfoot_des_pos.setZero();
  rfoot_des_pos[1] = -0.125;
  rfoot_des_quat.setIdentity();

  lfoot_des_pos.setZero();
  lfoot_des_pos[1] = 0.125;
  lfoot_des_quat.setIdentity();

  // Set Desired to current configuration except right palm must have identity orientation
  Eigen::Vector3d rpalm_des_pos;
  Eigen::Quaternion<double> rpalm_des_quat;
  ik_module.robot_model->getFrameWorldPose("rightPalm", rpalm_des_pos, rpalm_des_quat);
  rpalm_des_pos[0] += 0.25; 
  rpalm_des_pos[1] += 0.25; 
  rpalm_des_pos[2] += 0.1; 
  Eigen::AngleAxis<double> axis_angle;
  axis_angle.angle() = (M_PI/2.0);
  axis_angle.axis() = Eigen::Vector3d(0, 0, 1.0);
  rpalm_des_quat = axis_angle;

  // Set desired CoM to be 1m above the ground
  Eigen::Vector3d com_des_pos; com_des_pos.setZero();
  com_des_pos[2] = 1.0;


  // Set references ------------------------------------------------------------------------
  lfoot_task->setReference(lfoot_des_pos, lfoot_des_quat);
  rfoot_task->setReference(rfoot_des_pos, rfoot_des_quat);
  rpalm_task->setReference(rpalm_des_pos, rpalm_des_quat);
  com_task->setReference(com_des_pos);


  // Get Errors -----------------------------------------------------------------------------
  Eigen::VectorXd task_error;
  lfoot_task->getError(task_error);
  std::cout << "Left Foot Task Error = " << task_error.transpose() << std::endl;

  rfoot_task->getError(task_error);
  std::cout << "Right Foot Task Error = " << task_error.transpose() << std::endl;

  rpalm_task->getError(task_error);
  std::cout << "Right Palm Task Error = " << task_error.transpose() << std::endl;

  task_stack_priority_1->getError(task_error);
  std::cout << "L/R Foot Task Stack Error = " << task_error.transpose() << std::endl;

  ik_module.addTasktoHierarchy(task_stack_priority_1);
  ik_module.addTasktoHierarchy(task_stack_priority_2);


  int solve_result;
  bool inertia_weighted = true;
  double error_norm;
  Eigen::VectorXd q_sol = Eigen::VectorXd::Zero(ik_module.robot_model->getDimQdot());

  ik_module.prepareIKDataStrcutures();
  ik_module.solveIK(solve_result, error_norm, q_sol, inertia_weighted);


}

int main(int argc, char ** argv){
  testIK_module();
  return 0;
}