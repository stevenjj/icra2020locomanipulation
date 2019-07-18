#include <avatar_locomanipulation/collision_environment/collision_environment.h>


int main(int argc, char ** argv){

	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

	// Initialize Valkyrie RobotModel
	std::shared_ptr<RobotModel> valkyrie(new RobotModel(filename, meshDir, srdf_filename) );

	// X dimensional state vectors
  Eigen::VectorXd q_init;

  // Set origin at 0.0
  Eigen::VectorXd q_start = Eigen::VectorXd::Zero(valkyrie->getDimQ());
  double theta = 0.0;
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
  q_start[valkyrie->getJointIndex("rightAnklePitch")] = -0.3;

  // Puts the rightPalm in collision with the pelvis
  q_start[valkyrie->getJointIndex("rightShoulderPitch")] = -0.11;//-0.2;
  q_start[valkyrie->getJointIndex("rightShoulderRoll")] = 1.12;//1.1;
  q_start[valkyrie->getJointIndex("rightShoulderYaw")] = 1.5;//1.1;
  q_start[valkyrie->getJointIndex("rightElbowPitch")] = 1.66; //0.4;
  q_start[valkyrie->getJointIndex("rightForearmYaw")] = 0.0;

  q_start[valkyrie->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie->getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;

	// Update Robot Kinematics
  valkyrie->updateFullKinematics(q_init);
  valkyrie->updateGeometry(q_init);

  // Create the collision environment
  std::shared_ptr<CollisionEnvironment> collision;
  collision = std::shared_ptr<CollisionEnvironment>(new CollisionEnvironment(valkyrie) ) ;

  collision->directed_vectors.clear();
  collision->build_directed_vector_to_rhand();
  collision->self_collision_dx();
  for(int o=0; o<collision->directed_vectors.size(); ++o){
    std::cout << "collision->directed_vectors[o].from: " << collision->directed_vectors[o].from << std::endl;
    std::cout << "collision->directed_vectors[o].to: " << collision->directed_vectors[o].to << std::endl;
    std::cout << "collision->directed_vectors[o].magnitude: " << collision->directed_vectors[o].magnitude << std::endl;
    std::cout << "collision->directed_vectors[o].direction: \n" << collision->directed_vectors[o].direction << std::endl;
  }
}