#include <avatar_locomanipulation/feasibility/feasibility.hpp>

void feasibility::InverseKinematicsTop(const int & maxsteps){
  for(int ii=0; ii<maxsteps; ii++){
    valkyrie.updateFullKinematics(q_end);
    valkyrie.getFrameWorldPose("rightCOP_Frame", rfoot_cur_pos, rfoot_cur_ori);
    valkyrie.getFrameWorldPose("leftCOP_Frame", lfoot_cur_pos, lfoot_cur_ori);
    valkyrie.getFrameWorldPose("pelvis", pelvis_cur_pos, pelvis_cur_ori);
    valkyrie.computeCoMPos(q_end);

    // std::cout << com_des_pos << std::endl;

    ComputeTransError(rfoot_des_pos, rfoot_cur_pos, rfoot_pos_error);
    ComputeQuatError(rfoot_des_quat, rfoot_cur_ori, rfoot_ori_error);
    ComputeTransError(lfoot_des_pos, lfoot_cur_pos, lfoot_pos_error);
    ComputeQuatError(lfoot_des_quat, lfoot_cur_ori, lfoot_ori_error);
    ComputeTransError(com_des_pos, valkyrie.x_com, com_pos_error);
    ComputeQuatError(pelvis_des_quat, pelvis_cur_ori, pelvis_ori_error);


    // std::cout << pelvis_cur_ori << std::endl;

    task_error.head<3>() = rfoot_pos_error;
    task_error.segment<3>(3) = rfoot_ori_error;
    task_error.segment<3>(6) = lfoot_pos_error;
    task_error.segment<3>(9) = lfoot_ori_error;
    task_error.segment<3>(12) = com_pos_error;
    task_error.segment<3>(15) = pelvis_ori_error;

    // std::cout << task_error.transpose() << std::endl;

    ik_error_norm = task_error.norm();

    valkyrie.get6DTaskJacobian("rightCOP_Frame", J_rfoot);
    valkyrie.get6DTaskJacobian("leftCOP_Frame", J_lfoot);
    valkyrie.get6DTaskJacobian("pelvis", J_pelvis);
    valkyrie.computeCoMJacobian();

    // std::cout << valkyrie.J_com;

    J_task.topRows(6) = J_rfoot;
    J_task.row(6) = J_lfoot.row(0);
    J_task.row(7) = J_lfoot.row(1);
    J_task.row(8) = J_lfoot.row(2);
    J_task.row(9) = J_lfoot.row(3);
    J_task.row(10) = J_lfoot.row(4);
    J_task.row(11) = J_lfoot.row(5);
    J_task.row(12) = valkyrie.J_com.row(0);
    J_task.row(13) = valkyrie.J_com.row(1);
    J_task.row(14) = valkyrie.J_com.row(2);
    J_task.row(15) = J_pelvis.row(3);
    J_task.row(16) = J_pelvis.row(4);
    J_task.row(17) = J_pelvis.row(5);

    dq_change = svd->compute(J_task).solve(task_error);

    valkyrie.forwardIntegrate(q_end, 0.1*dq_change, q_end);

    for (int jj=0; jj<valkyrie.getDimQ(); jj++){
      q_end[jj] = clamp(q_end[jj], lower_lim[jj], upper_lim[jj]);
    }

    // std::cout << ii << " error norm = " << ik_error_norm << std::endl;
  }
  CreateData();
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

  q_start[valkyrie.getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;

  srand(time(0));
  Eigen::VectorXd q_rand(pinocchio::randomConfiguration(valkyrie.model, lower_lim, upper_lim ));

  q_start[valkyrie.getJointIndex("leftAnklePitch")] = q_rand[valkyrie.getJointIndex("leftAnklePitch")];
  q_start[valkyrie.getJointIndex("leftKneePitch")] = q_rand[valkyrie.getJointIndex("leftKneePitch")];
  q_start[valkyrie.getJointIndex("leftHipPitch")] = q_rand[valkyrie.getJointIndex("leftHipPitch")];

  q_start[valkyrie.getJointIndex("rightShoulderPitch")] = q_rand[valkyrie.getJointIndex("rightShoulderPitch")];
  q_start[valkyrie.getJointIndex("rightShoulderRoll")] = q_rand[valkyrie.getJointIndex("rightShoulderRoll")];
  q_start[valkyrie.getJointIndex("rightShoulderYaw")] = q_rand[valkyrie.getJointIndex("rightShoulderYaw")];
  q_start[valkyrie.getJointIndex("rightElbowPitch")] = q_rand[valkyrie.getJointIndex("rightElbowPitch")];
  q_start[valkyrie.getJointIndex("rightForearmYaw")] = q_rand[valkyrie.getJointIndex("rightForearmYaw")];
  q_start[valkyrie.getJointIndex("rightWristPitch")] = q_rand[valkyrie.getJointIndex("rightWristPitch")];
  q_start[valkyrie.getJointIndex("rightWristRoll")] = q_rand[valkyrie.getJointIndex("rightWristRoll")];

  q_start[valkyrie.getJointIndex("leftShoulderPitch")] = q_rand[valkyrie.getJointIndex("leftShoulderPitch")];
  q_start[valkyrie.getJointIndex("leftShoulderRoll")] = q_rand[valkyrie.getJointIndex("leftShoulderRoll")];
  q_start[valkyrie.getJointIndex("leftShoulderYaw")] = q_rand[valkyrie.getJointIndex("leftShoulderYaw")];
  q_start[valkyrie.getJointIndex("leftElbowPitch")] = q_rand[valkyrie.getJointIndex("leftElbowPitch")];
  q_start[valkyrie.getJointIndex("leftForearmYaw")] = q_rand[valkyrie.getJointIndex("leftForearmYaw")];
  q_start[valkyrie.getJointIndex("leftWristPitch")] = q_rand[valkyrie.getJointIndex("leftWristPitch")];
  q_start[valkyrie.getJointIndex("leftWristRoll")] = q_rand[valkyrie.getJointIndex("leftWristRoll")];


  q_start[valkyrie.getJointIndex("torsoYaw")] = q_rand[valkyrie.getJointIndex("torsoYaw")];
  q_start[valkyrie.getJointIndex("torsoPitch")] = q_rand[valkyrie.getJointIndex("torsoPitch")];
  q_start[valkyrie.getJointIndex("torsoRoll")] = q_rand[valkyrie.getJointIndex("torsoRoll")];

  q_start[2] = 2.0;
  q_start[6] = 1.0;

  // std::cout << q_start.transpose() << std::endl;

  q_end = q_start;

  rhand_data = Eigen::VectorXd::Zero(6);

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

  // std::cout << "lfoot_des_pos" << std::endl;
  // std::cout << lfoot_des_pos.transpose() << std::endl;

  lfoot_des_quat.setIdentity();
  double theta = rand()/((double) RAND_MAX)*0.75-.15;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0,0.0,1.0));
  lfoot_des_quat = aa;

  lfoot_cur_pos.setZero();
  lfoot_cur_ori.setIdentity();
  J_lfoot = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());

  lfoot_pos_error.setZero();
  //lfoot_ori_error.setZero();

  J_pelvis = Eigen::MatrixXd::Zero(6, valkyrie.getDimQdot());
  pelvis_des_quat.setIdentity();
  double theta_pel = theta/2.0;
  Eigen::AngleAxis<double> aa_pel(theta_pel, Eigen::Vector3d(0.0,0.0,1.0));
  pelvis_des_quat = aa_pel;

  // com_des_pos[0] = 0.0;
  // com_des_pos[1] = 0.0;
  com_des_pos[2] = 1.0;

  feasibility::initialize_foot_frames();
  feasibility::PointTrans(lfoot_des_pos, LFOF, LFOF_loc);
  feasibility::PointTrans(lfoot_des_pos, LFOB, LFOB_loc);
  feasibility::PointTrans(lfoot_des_pos, LFIF, LFIF_loc);
  feasibility::PointTrans(lfoot_des_pos, LFIB, LFIB_loc);
  feasibility::PointTrans(rfoot_des_pos, RFOF, RFOF_loc);
  feasibility::PointTrans(rfoot_des_pos, RFOB, RFOB_loc);
  feasibility::PointTrans(rfoot_des_pos, RFIF, RFIF_loc);
  feasibility::PointTrans(rfoot_des_pos, RFIB, RFIB_loc);

  std::vector<math_utils::Point> vec_points;
  vec_points.push_back(LFOF_loc);
  vec_points.push_back(LFOB_loc);
  vec_points.push_back(LFIF_loc);
  vec_points.push_back(LFIB_loc);
  vec_points.push_back(RFOF_loc);
  vec_points.push_back(RFOB_loc);
  vec_points.push_back(RFIF_loc);
  vec_points.push_back(RFIB_loc);

  std::vector<math_utils::Point> hull_vertices =  math_utils::convexHull(vec_points);

  // for(int i = 0; i < hull_vertices.size(); i++){
	// 	std::cout << "hull vertex: " << hull_vertices[i].x << ", " << hull_vertices[i].y << std::endl;
	// }

  for (int ii = 0; ii<hull_vertices.size()-1; ii++){
    alph = rand()/((double) RAND_MAX);
    rand_pt_x = rand_pt_x+(alph*hull_vertices[ii].x + (1-alph)*hull_vertices[ii+1].x);
    rand_pt_y = rand_pt_y+(alph*hull_vertices[ii].y + (1-alph)*hull_vertices[ii+1].y);
  }
    rand_pt_x = rand_pt_x/(hull_vertices.size()-1);
    rand_pt_y = rand_pt_y/(hull_vertices.size()-1);

    // std::cout << "CoM desired Position: " << rand_pt_x << " " << rand_pt_y << std::endl;

    com_des_pos[0] = rand_pt_x;
    com_des_pos[1] = rand_pt_y;

  com_pos_error.setZero();

  int task_dim = 18;
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
  upper_lim[valkyrie.getJointIndex("rightWristRoll")] = 0.62;
  upper_lim[valkyrie.getJointIndex("rightWristPitch")] = 0.36;
  //
  upper_lim[valkyrie.getJointIndex("leftShoulderPitch")] = 2.00;
  upper_lim[valkyrie.getJointIndex("leftShoulderRoll")] = 1.27;
  upper_lim[valkyrie.getJointIndex("leftShoulderYaw")] = 2.18;
  upper_lim[valkyrie.getJointIndex("leftElbowPitch")] = 0.12;
  upper_lim[valkyrie.getJointIndex("leftForearmYaw")] = 3.14;
  upper_lim[valkyrie.getJointIndex("leftWristRoll")] = 0.63;
  upper_lim[valkyrie.getJointIndex("leftWristPitch")] = 0.49;
  //
  upper_lim[valkyrie.getJointIndex("rightHipYaw")] = 0.41;
  upper_lim[valkyrie.getJointIndex("rightHipRoll")] = 0.47;
  upper_lim[valkyrie.getJointIndex("rightHipPitch")] = 1.62;
  upper_lim[valkyrie.getJointIndex("rightKneePitch")] = 2.06;
  upper_lim[valkyrie.getJointIndex("rightAnklePitch")] = 0.65;
  upper_lim[valkyrie.getJointIndex("rightAnkleRoll")] = 0.40;
  //
  upper_lim[valkyrie.getJointIndex("leftHipYaw")] = 1.10;
  upper_lim[valkyrie.getJointIndex("leftHipRoll")] = 0.55;
  upper_lim[valkyrie.getJointIndex("leftHipPitch")] = 1.62;
  upper_lim[valkyrie.getJointIndex("leftKneePitch")] = 2.06;
  upper_lim[valkyrie.getJointIndex("leftAnklePitch")] = 0.65;
  upper_lim[valkyrie.getJointIndex("leftAnkleRoll")] = 0.40;
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
  lower_lim[valkyrie.getJointIndex("rightWristRoll")] = -0.62;
  lower_lim[valkyrie.getJointIndex("rightWristPitch")] = -0.49;
  //
  lower_lim[valkyrie.getJointIndex("leftShoulderPitch")] = -2.85;
  lower_lim[valkyrie.getJointIndex("leftShoulderRoll")] = -1.52;
  lower_lim[valkyrie.getJointIndex("leftShoulderYaw")] = -3.10;
  lower_lim[valkyrie.getJointIndex("leftElbowPitch")] = -2.17;
  lower_lim[valkyrie.getJointIndex("leftForearmYaw")] = -2.02;
  lower_lim[valkyrie.getJointIndex("leftWristRoll")] = -0.62;
  lower_lim[valkyrie.getJointIndex("leftWristPitch")] = -0.36;
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

void feasibility::EigentoPoint(Eigen::Vector3d & eig_point, math_utils::Point & pt_point){
  pt_point.x = eig_point[0];
  pt_point.y = eig_point[1];
}

void feasibility::PointTrans(Eigen::Vector3d & eig_trans, math_utils::Point & pt_init_loc, math_utils::Point & pt_final_loc){
  pt_final_loc.x = pt_init_loc.x + eig_trans[0];
  pt_final_loc.y = pt_init_loc.y + eig_trans[1];
}

void feasibility::initialize_foot_frames(){
  LFOF.x = .1;
  LFOF.y = .05;

  LFOB.x = -.1;
  LFOB.y = .05;

  LFIB.x = -.1;
  LFIB.y = -.05;

  LFIF.x = .1;
  LFIF.y = -.05;

  RFOF.x = .1;
  RFOF.y = -.05;

  RFOB.x = -.1;
  RFOB.y = -.05;

  RFIB.x = -.1;
  RFIB.y = .05;

  RFIF.x = .1;
  RFIF.y = .05;
}

void feasibility::CreateData() {
  q_data = q_end;
  q_data[valkyrie.getJointIndex("torsoYaw")] = q_start[valkyrie.getJointIndex("torsoYaw")];
  q_data[valkyrie.getJointIndex("torsoPitch")] = q_start[valkyrie.getJointIndex("torsoPitch")];
  q_data[valkyrie.getJointIndex("torsoRoll")] = q_start[valkyrie.getJointIndex("torsoRoll")];

  q_data[valkyrie.getJointIndex("rightShoulderPitch")] = q_start[valkyrie.getJointIndex("rightShoulderPitch")];
  q_data[valkyrie.getJointIndex("rightShoulderRoll")] = q_start[valkyrie.getJointIndex("rightShoulderRoll")];
  q_data[valkyrie.getJointIndex("rightShoulderYaw")] = q_start[valkyrie.getJointIndex("rightShoulderYaw")];
  q_data[valkyrie.getJointIndex("rightElbowPitch")] = q_start[valkyrie.getJointIndex("rightElbowPitch")];
  q_data[valkyrie.getJointIndex("rightForearmYaw")] = q_start[valkyrie.getJointIndex("rightForearmYaw")];
  q_data[valkyrie.getJointIndex("rightWristRoll")] = q_start[valkyrie.getJointIndex("rightWristRoll")];
  q_data[valkyrie.getJointIndex("rightWristPitch")] = q_start[valkyrie.getJointIndex("rightWristPitch")];

  q_data[valkyrie.getJointIndex("leftShoulderPitch")] = q_start[valkyrie.getJointIndex("leftShoulderPitch")];
  q_data[valkyrie.getJointIndex("leftShoulderRoll")] = q_start[valkyrie.getJointIndex("leftShoulderRoll")];
  q_data[valkyrie.getJointIndex("leftShoulderYaw")] = q_start[valkyrie.getJointIndex("leftShoulderYaw")];
  q_data[valkyrie.getJointIndex("leftElbowPitch")] = q_start[valkyrie.getJointIndex("leftElbowPitch")];
  q_data[valkyrie.getJointIndex("leftForearmYaw")] = q_start[valkyrie.getJointIndex("leftForearmYaw")];
  q_data[valkyrie.getJointIndex("leftWristRoll")] = q_start[valkyrie.getJointIndex("leftWristRoll")];
  q_data[valkyrie.getJointIndex("leftWristPitch")] = q_start[valkyrie.getJointIndex("leftWristPitch")];

  q_data[valkyrie.getJointIndex("lowerNeckPitch")] = 0.0;
  q_data[valkyrie.getJointIndex("neckYaw")] = 0.0;
  q_data[valkyrie.getJointIndex("upperNeckPitch")] = -0.0;
}

void feasibility::generateRandomArm(){
  double theta = M_PI/4.0;
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

  q_start[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;
  Eigen::VectorXd q_rand(pinocchio::randomConfiguration(valkyrie.model, lower_lim, upper_lim ));
  q_start[valkyrie.getJointIndex("rightShoulderPitch")] = q_rand[valkyrie.getJointIndex("rightShoulderPitch")];
  q_start[valkyrie.getJointIndex("rightShoulderRoll")] = q_rand[valkyrie.getJointIndex("rightShoulderRoll")];
  q_start[valkyrie.getJointIndex("rightShoulderYaw")] = q_rand[valkyrie.getJointIndex("rightShoulderYaw")];
  q_start[valkyrie.getJointIndex("rightElbowPitch")] = q_rand[valkyrie.getJointIndex("rightElbowPitch")];
  q_start[valkyrie.getJointIndex("rightForearmYaw")] = q_rand[valkyrie.getJointIndex("rightForearmYaw")];


}

// int main(int argc, char ** argv) {
//   feasibility fs;
//   fs.InverseKinematicsTop(10);
//   return 0;
//   }

//rosrun avatar_locomanipulation feasibility
