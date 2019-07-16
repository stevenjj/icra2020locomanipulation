#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <avatar_locomanipulation/tasks/task.hpp>
#include <avatar_locomanipulation/tasks/task_com.hpp>
#include <avatar_locomanipulation/tasks/task_6dpose.hpp>
#include <avatar_locomanipulation/tasks/task_3dorientation.hpp>
#include <avatar_locomanipulation/tasks/task_joint_config.hpp>
#include <avatar_locomanipulation/tasks/task_stack.hpp>
#include <avatar_locomanipulation/tasks/task_6dcontact_normal.hpp>

#include <iostream>

void get_initial_configuration(const std::shared_ptr<ValkyrieModel> & valkyrie, Eigen::VectorXd & q_start){
  q_start = Eigen::VectorXd::Zero(valkyrie->getDimQ());

  double theta = 0.0; //M_PI/4.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q
  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[valkyrie->getJointIndex("leftHipPitch")] = -0.3;
  q_start[valkyrie->getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie->getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie->getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie->getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie->getJointIndex("rightAnklePitch")] = 0.0;//-0.3;

  q_start[valkyrie->getJointIndex("rightShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie->getJointIndex("rightElbowPitch")] = 0.4;
  q_start[valkyrie->getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie->getJointIndex("leftForearmYaw")] = 1.5;

  std::cout << "starting configuration " << std::endl;
  std::cout << q_start.transpose() << std::endl;
}

int main(int argc, char ** argv){
	std::shared_ptr<ValkyrieModel> valkyrie_model(new ValkyrieModel());
  Eigen::VectorXd q_start;
  Eigen::VectorXd qdot_start = Eigen::VectorXd::Zero(valkyrie_model->getDimQdot());
  Eigen::VectorXd qddot_start = Eigen::VectorXd::Zero(valkyrie_model->getDimQdot());
  get_initial_configuration(valkyrie_model, q_start);

	Task task(valkyrie_model);
  std::shared_ptr<Task> com_task(new TaskCOM(valkyrie_model));
  std::shared_ptr<Task> lfoot_task(new Task6DPose(valkyrie_model, "leftCOP_Frame"));
  std::shared_ptr<Task> pelvis_ori_task(new Task3DOrientation(valkyrie_model, "pelvis"));
  std::shared_ptr<Task> joint_config_task(new TaskJointConfig(valkyrie_model, {"leftHipYaw", "leftHipRoll"}));

  Eigen::Vector3d plane_normal(0,0,10);
  Eigen::Vector3d plane_center(5,0,0);  
  std::shared_ptr<Task> rfoot_planar_contact_task(new Task6DContactNormalTask(valkyrie_model, "rightCOP_Frame", plane_normal, plane_center));


  std::shared_ptr<Task> task_stack(new TaskStack(valkyrie_model, {com_task, lfoot_task, pelvis_ori_task, joint_config_task}));


	std::cout << "main task dim = " << task.task_dim << std::endl;
	std::cout << "com task dim = " << com_task->task_dim << std::endl;
  std::cout << "left foot task dim = " << lfoot_task->task_dim << std::endl;
  std::cout << "pelvis ori task dim = " << pelvis_ori_task->task_dim << std::endl;
  std::cout << "joint config task dim = " << joint_config_task->task_dim << std::endl;
  std::cout << "task stack dim = " << task_stack->task_dim << std::endl;

	// Update Kinematics and Jacobians
	valkyrie_model->updateFullKinematics(q_start);
	// Compute CoM Jacobian
	valkyrie_model->computeCoMJacobian();

	Eigen::MatrixXd J_com = Eigen::MatrixXd::Zero(com_task->task_dim, valkyrie_model->getDimQdot());
  Eigen::MatrixXd J_lf = Eigen::MatrixXd::Zero(lfoot_task->task_dim, valkyrie_model->getDimQdot());
  Eigen::MatrixXd J_pelvis_ori = Eigen::MatrixXd::Zero(pelvis_ori_task->task_dim, valkyrie_model->getDimQdot());
  Eigen::MatrixXd J_joint_config = Eigen::MatrixXd::Zero(joint_config_task->task_dim, valkyrie_model->getDimQdot());
  Eigen::MatrixXd J_stack = Eigen::MatrixXd::Zero(task_stack->task_dim, valkyrie_model->getDimQdot());


  rfoot_planar_contact_task->computeError();

  com_task->getTaskJacobian(J_com);
  std::cout << "COM Jacobian" << std::endl;
  std::cout << J_com << std::endl;

  lfoot_task->getTaskJacobian(J_lf);
  std::cout << "Left Foot Jacobian" << std::endl;
  std::cout << J_lf << std::endl;

  pelvis_ori_task->getTaskJacobian(J_pelvis_ori);
  std::cout << "pelvis Ori Jacobian" << std::endl;
  std::cout << J_pelvis_ori << std::endl;

  joint_config_task->getTaskJacobian(J_joint_config);
  std::cout << "Joint Config Jacobian" << std::endl;
  std::cout << J_joint_config << std::endl;  

  task_stack->getTaskJacobian(J_stack);
  std::cout << "Stacked Jacobian" << std::endl;
  std::cout << J_stack << std::endl;  

	// Compute Derivatives
	valkyrie_model->updateKinematicsDerivatives(q_start, qdot_start, qddot_start);
  valkyrie_model->computeCoMJacobianDot(q_start, qdot_start);




	return 0;
}