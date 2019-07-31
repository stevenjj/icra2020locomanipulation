#include <avatar_locomanipulation/models/robot_model.hpp>


pinocchio::fcl::Quaternion3f makeQuat(double w, double x, double y, double z){
  pinocchio::fcl::Quaternion3f q;
  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  return q;
}



int main(int argc, char ** argv){

//------------------------ Define the Valkyrie RobotModel
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
//------------------------ End Define the Valkyrie RobotModel

   

//------------------------ Define the Cart RobotModel
  filename = THIS_PACKAGE_PATH"models/test_cart.urdf";
  meshDir  = THIS_PACKAGE_PATH"models/cart/";

  // Initialize Cart RobotModel
  std::shared_ptr<RobotModel> cart(new RobotModel(filename, meshDir) );

   // Define the configuration of the cart
  Eigen::VectorXd cart_config;
  cart_config = Eigen::VectorXd::Zero(cart->getDimQ());
  cart_config[0] = 0.11111;  cart_config[1] = 0.22222;  cart_config[2] = 0.33333;
  double theta1 = 0;//M_PI/4.0; 
  Eigen::AngleAxis<double> bb(theta1, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left  
  Eigen::Quaternion<double> quat_init; quat_init =  bb;
  cart_config[3] = quat_init.x();// 0.0;  
  cart_config[4] = quat_init.y(); //0.0;
  cart_config[5] = quat_init.z(); //sin(theta/2.0);
  cart_config[6] = quat_init.w(); //cos(theta/2.0);

  cart->q_current = cart_config;

  cart->enableUpdateGeomOnKinematicsUpdate(true);
  cart->updateFullKinematics(cart_config);
//------------------------- End Define the Cart RobotModel



//------------------------- Define the empty Appended RobotModel
  // This is the default constructor, meaning that the appended model has nothing
  	// No data, geomData, etc.
std::shared_ptr<RobotModel> appended(new RobotModel() );
//------------------------- End Define the empty Appended RobotModel



//------------------------- Append the Models
	// This will give appended a model and geomModel
  pinocchio::appendModel(valkyrie->model, cart->model, valkyrie->geomModel, cart->geomModel, 0, pinocchio::SE3::Identity(), appended->model, appended->geomModel);
//------------------------- End Append the Models

  // What does appended init actually do
  // Can we getGeometryName or something to that effect
  	// i.e how can we figure out which index of q represents which body

 
  // appended_initialization will do the following:
  	// define a data, geomData, and some of the common matrices
  	// set the length of q_current
  	// set the updateGeomWithKinematics = false
  appended->appended_initialization();


  // At this point, we will find that the q_current has the same size as getDimQ() will return
  // for all three RobotModels


  // appended->q_current is still empty, but we need to ensure we fill it in a smart way
  std::cout << "appended->geomModel:\n" << appended->geomModel << std::endl;
  for (int k=0 ; k<appended->model.frames.size() ; ++k){
    std::cout << "frame:" << k << " " << appended->model.frames[k].name << " : " << appended->data->oMf[k].translation().transpose() << std::endl;
  }


  // Fill appended->q_current
 

  // If we want to be able to update the geometry placements and then computeDistance on the
  //	new config, we need to:
  appended->enableUpdateGeomOnKinematicsUpdate(true);


}



// STUFF USED TO DEBUG


 // std::cout << "cart->getDimQ(): " << cart->getDimQ() << std::endl;
  // std::cout << "valkyrie->getDimQ(): " << valkyrie->getDimQ() << std::endl;
  // std::cout << "appended->getDimQ(): " << appended->getDimQ() << std::endl;

 // std::cout << "cart->q_current.size(): " << cart->q_current.size() << std::endl;
  // std::cout << "valkyrie->q_current.size(): " << valkyrie->q_current.size() << std::endl;
  // std::cout << "appended->q_current.size(): " << appended->q_current.size() << std::endl;

