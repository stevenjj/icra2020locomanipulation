#include <avatar_locomanipulation/collision_environment/collision_environment.h>

int main(int argc, char ** argv){
  std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

	// Initialize Valkyrie RobotModel
	std::shared_ptr<RobotModel> valkyrie(new RobotModel(filename, meshDir, srdf_filename) );

   // Define the configuration of Val
  Eigen::VectorXd q_start;
  q_start = Eigen::VectorXd::Zero(valkyrie->getDimQ());

  double theta = 0;//M_PI/4.0;
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

  q_start[valkyrie->getJointIndex("rightShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("rightShoulderRoll")] = 1.1;
  q_start[valkyrie->getJointIndex("rightElbowPitch")] = 0.4;
  q_start[valkyrie->getJointIndex("rightForearmYaw")] = 1.5;

  q_start[valkyrie->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie->getJointIndex("leftForearmYaw")] = 1.5;

  valkyrie->q_current = q_start;

  valkyrie->enableUpdateGeomOnKinematicsUpdate(true);
  valkyrie->updateFullKinematics(q_start);

  std::shared_ptr<CollisionEnvironment> collision(new CollisionEnvironment(valkyrie) );


  std::vector<Eigen::Vector3d> point_list;
  Eigen::Vector3d point;
  point << 1., 1., 1.;
  point_list.push_back(point);
  point[0] = 0.0; point[1] = -0.4; point[2] = 1.2; 
  point_list.push_back(point);
  point[0] = 0.0; point[1] = 0.4; point[2] = 1.2; 
  point_list.push_back(point);

  collision->build_point_list_directed_vectors(point_list, q_start);
  std::cout << "collision->directed_vectors.size(): " << collision->directed_vectors.size() << std::endl;;
  //   for(int i=0; i<collision->directed_vectors.size(); ++i){
  //   std::cout << "collision->directed_vectors[i].from: " << collision->directed_vectors[i].from << std::endl;
  //   std::cout << "collision->directed_vectors[i].to: " << collision->directed_vectors[i].to << std::endl;
  //   std::cout << "collision->directed_vectors[i].magnitude: " << collision->directed_vectors[i].magnitude << std::endl;
  //   std::cout << "collision->directed_vectors[i].direction: \n" << collision->directed_vectors[i].direction << std::endl;
  // }

  collision->set_safety_distance_normal(0.2);

  double dx;
  dx = collision->get_collision_potential();
  std::cout << "collision->directed_vectors[collision->closest].magnitude: " << collision->directed_vectors[collision->closest].magnitude << std::endl;
  std::cout << "collision->directed_vectors[collision->closest].from: " << collision->directed_vectors[collision->closest].from << std::endl;
  std::cout << "collision->directed_vectors[collision->closest].to: " << collision->directed_vectors[collision->closest].to << std::endl;
  std::cout << "dx = " << dx << std::endl;
}


