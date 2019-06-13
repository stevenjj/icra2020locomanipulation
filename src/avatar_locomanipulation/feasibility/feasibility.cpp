#include <avatar_locomanipulation/feasibility/feasibility.hpp>

void feasibility::InverseKinematicsTop(const int & maxsteps){
  for(int ii=0; ii<maxsteps; ii++){
    valkyrie.updateFullKinematics(q_end);
    valkyrie.getFrameWorldPose("rightCOP_Frame", rfoot_cur_pos, rfoot_cur_ori);
    valkyrie.getFrameWorldPose("leftCOP_Frame", lfoot_cur_pos, lfoot_cur_ori);
    valkyrie.computeCoMPos(q_end);

    // std::cout << com_des_pos << std::endl;

    ComputeTransError(rfoot_des_pos, rfoot_cur_pos, rfoot_pos_error);
    ComputeQuatError(rfoot_des_quat, rfoot_cur_ori, rfoot_ori_error);
    ComputeTransError(lfoot_des_pos, lfoot_cur_pos, lfoot_pos_error);
    ComputeQuatError(lfoot_des_quat, lfoot_cur_ori, lfoot_ori_error);
    ComputeTransError(com_des_pos, valkyrie.x_com, com_pos_error);


    //std::cout << lfoot_cur_pos.transpose() << std::endl;

    task_error.head<3>() = rfoot_pos_error;
    task_error.segment<3>(3) = rfoot_ori_error;
    task_error.segment<3>(6) = lfoot_pos_error;
    task_error.segment<3>(9) = lfoot_ori_error;
    task_error.segment<3>(12) = com_pos_error;

    // std::cout << task_error.transpose() << std::endl;

    ik_error_norm = task_error.norm();

    valkyrie.get6DTaskJacobian("rightCOP_Frame", J_rfoot);
    valkyrie.get6DTaskJacobian("leftCOP_Frame", J_lfoot);
    valkyrie.computeCoMJacobian();

    // std::cout << valkyrie.J_com;

    J_task.topRows(6) = J_rfoot;
    J_task.row(6) = J_lfoot.row(0);
    J_task.row(7) = J_lfoot.row(1);
    J_task.row(8) = J_lfoot.row(2);
    J_task.row(9) = J_lfoot.row(3);
    J_task.row(10) = J_lfoot.row(4);
    J_task.row(11) = J_lfoot.row(5);
    J_task.bottomRows(3) = valkyrie.J_com;

    dq_change = svd->compute(J_task).solve(task_error);

    valkyrie.forwardIntegrate(q_end, dq_change, q_end);

    for (int jj=0; jj<valkyrie.getDimQ(); jj++){
      q_end[jj] = clamp(q_end[jj], lower_lim[jj], upper_lim[jj]);
    }

    std::cout << ii << " error norm = " << ik_error_norm << std::endl;
  }
}

void feasibility::ComputeTransError(const Eigen::Vector3d & des, const Eigen::Vector3d & current, Eigen::Vector3d & error){
  error = des - current;
}

void feasibility::ComputeQuatError(const Eigen::Quaternion<double> & des,
    							const Eigen::Quaternion<double> & current,
    							Eigen::Vector3d & error){
                    axis_angle = des*current.inverse();
                    error = axis_angle.axis() * axis_angle.angle();
                  }

void feasibility::initialize_configuration_top(){
  // initialized configurations.
  q_start = Eigen::VectorXd::Zero(valkyrie.getDimQ());
	q_end = Eigen::VectorXd::Zero(valkyrie.getDimQ());
	dq_change = Eigen::VectorXd::Zero(valkyrie.getDimQdot());

  srand(time(0));
  q_start = pinocchio::randomConfiguration(valkyrie.model, lower_lim, upper_lim );
  q_start[valkyrie.getJointIndex("neckYaw")] = 0.0;
  q_start[valkyrie.getJointIndex("lowerNeckPitch")] = 0.0;
  q_start[valkyrie.getJointIndex("upperNeckPitch")] = 0.0;
  //
  q_start[0] = 0;
  q_start[1] = 0;
  q_start[2] = 1;


  q_start[3] = 0;
  q_start[4] = 0;
  q_start[5] = 0;
  q_start[6] = 1;

  std::cout << q_start.transpose() << std::endl;

  q_end = q_start;

  valkyrie.updateFullKinematics(q_end);
}

void feasibility::initialize_desired_top(){
  srand(time(0));
  rfoot_des_pos.setZero();
  rfoot_des_quat.setIdentity();

  rfoot_cur_pos.setZero();
  rfoot_cur_ori.setIdentity();
  J_rfoot = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());

  rfoot_pos_error.setZero();
  rfoot_ori_error.setZero();

  lfoot_des_pos.setZero();

  lfoot_des_pos[0] = rand()/((double) RAND_MAX)*0.5-0.15;
  lfoot_des_pos[1] = rand()/((double) RAND_MAX)*0.2+0.2;

  std::cout << "lfoot_des_pos" << std::endl;
  std::cout << lfoot_des_pos.transpose() << std::endl;
  lfoot_des_quat.setIdentity();
  double theta = rand()/((double) RAND_MAX)*0.75-.15;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0,0.0,1.0));
  lfoot_des_quat = aa;

  lfoot_cur_pos.setZero();
  lfoot_cur_ori.setIdentity();
  J_lfoot = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());

  lfoot_pos_error.setZero();
  //lfoot_ori_error.setZero();

  com_des_pos[0] = 0.0;
  com_des_pos[1] = 0.0;
  com_des_pos[2] = 1.0;

  com_pos_error.setZero();

  int task_dim = 15;
  J_task = Eigen::MatrixXd::Zero(task_dim, valkyrie.getDimQdot());
  task_error = Eigen::VectorXd::Zero(task_dim);
  svd = std::unique_ptr< Eigen::SVD_SOLVER<Eigen::MatrixXd> >( new Eigen::SVD_SOLVER<Eigen::MatrixXd>(J_task.rows(), valkyrie.getDimQdot(), svdOptions) );

  const double svd_thresh = 1e-4;
  svd->setThreshold(svd_thresh);
  }

feasibility::feasibility(){
  initialize_lower_limits();
  initialize_upper_limits();
  initialize_configuration_top();
  initialize_desired_top();
}

feasibility::~feasibility(){
}

void feasibility::initialize_upper_limits(){
  upper_lim = 10*Eigen::VectorXd::Ones(valkyrie.getDimQ());
  upper_lim[valkyrie.getJointIndex("torsoYaw")] = 1.18;
  upper_lim[valkyrie.getJointIndex("torsoPitch")] = 0.67;
  upper_lim[valkyrie.getJointIndex("torsoRoll")] = 0.26;
  upper_lim[valkyrie.getJointIndex("lowerNeckPitch")] = 1.16;
  upper_lim[valkyrie.getJointIndex("neckYaw")] = 1.05;
  upper_lim[valkyrie.getJointIndex("upperNeckPitch")] = 0.0;
  //
  upper_lim[valkyrie.getJointIndex("rightShoulderPitch")] = 2.00;
  upper_lim[valkyrie.getJointIndex("rightShoulderRoll")] = 1.52;
  upper_lim[valkyrie.getJointIndex("rightShoulderYaw")] = 2.18;
  upper_lim[valkyrie.getJointIndex("rightElbowPitch")] = 2.17;
  upper_lim[valkyrie.getJointIndex("rightForearmYaw")] = 3.14;
  // upper_lim[valkyrie.getJointIndex("rightWristRoll")] = 0.62;
  // upper_lim[valkyrie.getJointIndex("rightWristPitch")] = 0.36;
  //
  upper_lim[valkyrie.getJointIndex("leftShoulderPitch")] = 2.00;
  upper_lim[valkyrie.getJointIndex("leftShoulderRoll")] = 1.27;
  upper_lim[valkyrie.getJointIndex("leftShoulderYaw")] = 2.18;
  upper_lim[valkyrie.getJointIndex("leftElbowPitch")] = 0.12;
  upper_lim[valkyrie.getJointIndex("leftForearmYaw")] = 3.14;
  // upper_lim[valkyrie.getJointIndex("leftWristRoll")] = 0.63;
  // upper_lim[valkyrie.getJointIndex("leftWristPitch")] = 0.49;
  //
  upper_lim[valkyrie.getJointIndex("rightHipYaw")] = 0.41;
  upper_lim[valkyrie.getJointIndex("rightHipRoll")] = 0.47;
  upper_lim[valkyrie.getJointIndex("rightHipPitch")] = 1.62;
  upper_lim[valkyrie.getJointIndex("rightKneePitch")] = 2.06;
  upper_lim[valkyrie.getJointIndex("rightAnklePitch")] = 0.65;
  upper_lim[valkyrie.getJointIndex("rightAnkleRoll")] = 0.40;
  //
  upper_lim[valkyrie.getJointIndex("rightHipYaw")] = 1.10;
  upper_lim[valkyrie.getJointIndex("rightHipRoll")] = 0.55;
  upper_lim[valkyrie.getJointIndex("rightHipPitch")] = 1.62;
  upper_lim[valkyrie.getJointIndex("rightKneePitch")] = 2.06;
  upper_lim[valkyrie.getJointIndex("rightAnklePitch")] = 0.65;
  upper_lim[valkyrie.getJointIndex("rightAnkleRoll")] = 0.40;
}

void feasibility::initialize_lower_limits(){
  lower_lim = -10 * Eigen::VectorXd::Ones(valkyrie.getDimQ());
  lower_lim[valkyrie.getJointIndex("torsoYaw")] = -1.33;
  lower_lim[valkyrie.getJointIndex("torsoPitch")] = -0.13;
  lower_lim[valkyrie.getJointIndex("torsoRoll")] = -0.23;
  lower_lim[valkyrie.getJointIndex("lowerNeckPitch")] = 0.0;
  lower_lim[valkyrie.getJointIndex("neckYaw")] = -1.05;
  lower_lim[valkyrie.getJointIndex("upperNeckPitch")] = -0.87;
  //
  lower_lim[valkyrie.getJointIndex("rightShoulderPitch")] = -2.85;
  lower_lim[valkyrie.getJointIndex("rightShoulderRoll")] = -1.27;
  lower_lim[valkyrie.getJointIndex("rightShoulderYaw")] = -3.10;
  lower_lim[valkyrie.getJointIndex("rightElbowPitch")] = -0.12;
  lower_lim[valkyrie.getJointIndex("rightForearmYaw")] = -2.02;
  // lower_lim[valkyrie.getJointIndex("rightWristRoll")] = -0.62;
  // lower_lim[valkyrie.getJointIndex("rightWristPitch")] = -0.49;
  //
  lower_lim[valkyrie.getJointIndex("leftShoulderPitch")] = -2.85;
  lower_lim[valkyrie.getJointIndex("leftShoulderRoll")] = -1.52;
  lower_lim[valkyrie.getJointIndex("leftShoulderYaw")] = -3.10;
  lower_lim[valkyrie.getJointIndex("leftElbowPitch")] = -2.17;
  lower_lim[valkyrie.getJointIndex("leftForearmYaw")] = -2.02;
  // lower_lim[valkyrie.getJointIndex("leftWristRoll")] = -0.62;
  // lower_lim[valkyrie.getJointIndex("leftWristPitch")] = -0.36;
  //
  lower_lim[valkyrie.getJointIndex("rightHipYaw")] = -1.10;
  lower_lim[valkyrie.getJointIndex("rightHipRoll")] = -0.55;
  lower_lim[valkyrie.getJointIndex("rightHipPitch")] = -2.42;
  lower_lim[valkyrie.getJointIndex("rightKneePitch")] = -0.08;
  lower_lim[valkyrie.getJointIndex("rightAnklePitch")] = -0.93;
  lower_lim[valkyrie.getJointIndex("rightAnkleRoll")] = -0.40;

  lower_lim[valkyrie.getJointIndex("leftHipYaw")] = -0.41;
  lower_lim[valkyrie.getJointIndex("leftHipRoll")] = -0.47;
  lower_lim[valkyrie.getJointIndex("leftHipPitch")] = -2.42;
  lower_lim[valkyrie.getJointIndex("leftKneePitch")] = -0.08;
  lower_lim[valkyrie.getJointIndex("leftAnklePitch")] = -0.93;
  lower_lim[valkyrie.getJointIndex("leftAnkleRoll")] = -0.40;
}

double feasibility::clamp(const double & s_in, double lo, double hi){
    if (s_in < lo){
        return lo;
    }
    else if(s_in > hi){
        return hi;
    }else{
        return s_in;
    }

}

// int main(int argc, char ** argv) {
//   feasibility fs;
//   fs.InverseKinematicsTop(10);
//   return 0;
//   }

//rosrun avatar_locomanipulation feasibility
