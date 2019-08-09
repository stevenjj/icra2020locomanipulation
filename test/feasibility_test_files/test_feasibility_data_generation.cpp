#include <avatar_locomanipulation/feasibility/feasibility_data_generator.hpp>


void initialize_config(Eigen::VectorXd & q_init, std::shared_ptr<RobotModel> & robot_model){
  // X dimensional state vectors
  Eigen::VectorXd q_start;

  // std::cout << "robot_model->getDimQ(): " << robot_model->getDimQ() << std::endl;
  // Set origin at 0.0
  q_start = Eigen::VectorXd::Zero(robot_model->getDimQ());

  double theta = 0.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q
  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[robot_model->getJointIndex("leftHipPitch")] = -0.3;
  q_start[robot_model->getJointIndex("rightHipPitch")] = -0.3;
  q_start[robot_model->getJointIndex("leftKneePitch")] = 0.6;
  q_start[robot_model->getJointIndex("rightKneePitch")] = 0.6;
  q_start[robot_model->getJointIndex("leftAnklePitch")] = -0.3;
  q_start[robot_model->getJointIndex("rightAnklePitch")] = 0.0;//-0.3;

  q_start[robot_model->getJointIndex("rightShoulderPitch")] = 0.2;
  q_start[robot_model->getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[robot_model->getJointIndex("rightElbowPitch")] = 1.0 ; //0.4;
  q_start[robot_model->getJointIndex("rightForearmYaw")] = 1.5;

  q_start[robot_model->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[robot_model->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[robot_model->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[robot_model->getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}

int main(int argc, char ** argv){
  std::cout << "[Testing FeasibilityDataGenerator]" << std::endl;

  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));
  Eigen::VectorXd q_ik_start;
  initialize_config(q_ik_start, robot_model);

  // Initialize feasibility data generator
  FeasibilityDataGenerator feas_data_gen;

  // Set the robot model
  feas_data_gen.setRobotModel(robot_model);
  // Set the initial IK configuration
  feas_data_gen.setStartingIKConfig(q_ik_start);

  // Set the seed
  unsigned int seed_number = 1;
  feas_data_gen.initializeSeed(seed_number);

  for(int i = 0; i < 10; i++){
    // set random starting configuration
    feas_data_gen.randomizeStartingConfiguration();   
  }


  return 0;
}