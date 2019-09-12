#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Multibody
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
// Algorithms
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp" // Jacobian Frame Computation 
// Parsers
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
// Spatial
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"

// Standard
#include <math.h>
#include <map>
#include <iostream>
#include <boost/shared_ptr.hpp>


int main(int argc, char ** argv){
	// models and geomModels
	pinocchio::Model robot_model, cart_model, appended_model;
  pinocchio::GeometryModel robot_geomModel, cart_geomModel, appended_geomModel;
  // data and geomdata
  std::unique_ptr<pinocchio::Data> robot_data, cart_data, appended_data;
  std::unique_ptr<pinocchio::GeometryData> robot_geomData, cart_geomData, appended_geomData;

  // Define the Robot--------
	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";
  // build model and geomModel
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),robot_model);
  pinocchio::urdf::buildGeom(robot_model, filename, pinocchio::COLLISION, robot_geomModel, meshDir );
  // Add and remove collision pairs
  robot_geomModel.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(robot_model, robot_geomModel, srdf_filename, false);
  // Define the data and geomData
  robot_data = std::unique_ptr<pinocchio::Data>(new pinocchio::Data(robot_model));
	robot_geomData = std::unique_ptr<pinocchio::GeometryData>(new pinocchio::GeometryData(robot_geomModel));
	// Define the configuration of Robot
  Eigen::VectorXd q_robot;
  q_robot = Eigen::VectorXd::Zero(robot_model.nq);

  double theta = 0;//M_PI/4.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_robot[3] = init_quat.x(); q_robot[4] = init_quat.y(); q_robot[5] = init_quat.z(); q_robot[6] = init_quat.w(); // Set up the quaternion in q

  q_robot[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_robot[7 + robot_model.getJointId("leftHipPitch") - 2] = -0.3;
  q_robot[7 + robot_model.getJointId("rightHipPitch") - 2] = -0.3;
  q_robot[7 + robot_model.getJointId("leftKneePitch") - 2] = 0.6;
  q_robot[7 + robot_model.getJointId("rightKneePitch") - 2] = 0.6;
  q_robot[7 + robot_model.getJointId("leftAnklePitch") - 2] = -0.3;
  q_robot[7 + robot_model.getJointId("rightAnklePitch") - 2] = -0.3;

  q_robot[7 + robot_model.getJointId("rightShoulderPitch") - 2] = -0.2;
  q_robot[7 + robot_model.getJointId("rightShoulderRoll") - 2] = 1.1;
  q_robot[7 + robot_model.getJointId("rightElbowPitch") - 2] = 0.4;
  q_robot[7 + robot_model.getJointId("rightForearmYaw") - 2] = 1.5;

  q_robot[7 + robot_model.getJointId("leftShoulderPitch") - 2] = -0.2;
  q_robot[7 + robot_model.getJointId("leftShoulderRoll") - 2] = -1.1;
  q_robot[7 + robot_model.getJointId("leftElbowPitch") - 2] = -0.4;
  q_robot[7 + robot_model.getJointId("leftForearmYaw") - 2] = 1.5;

	// Define the Cart--------
	filename = THIS_PACKAGE_PATH"models/test_cart.urdf";
  meshDir  = THIS_PACKAGE_PATH"models/cart/";
  // build model and geomModel
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),cart_model);
  pinocchio::urdf::buildGeom(cart_model, filename, pinocchio::COLLISION, cart_geomModel, meshDir );

  //------ Suggested Fix
  // Apply a prefix
  std::string prefix ("cart/");
  for (pinocchio::JointIndex i = 1; i < cart_model.joints.size(); ++i) {
    cart_model.names[i] = prefix + cart_model.names[i];
  }
  for (pinocchio::FrameIndex i = 0; i < cart_model.frames.size(); ++i) {
    ::pinocchio::Frame& f = cart_model.frames[i];
    f.name = prefix + f.name;
  }
  //----------------------
  // Add and remove collision pairs
  cart_geomModel.addAllCollisionPairs();
  // Define the data and geomData
  cart_data = std::unique_ptr<pinocchio::Data>(new pinocchio::Data(cart_model));
	cart_geomData = std::unique_ptr<pinocchio::GeometryData>(new pinocchio::GeometryData(cart_geomModel));
	// Define the configuration of the cart
  Eigen::VectorXd q_cart;
  q_cart = Eigen::VectorXd::Zero(cart_model.nq);
  q_cart[0] = -0.05;  q_cart[1] = 0.0;  q_cart[2] = 0.0;
  // q_cart[0] = -0.06;  q_cart[1] = -0.0085;  q_cart[2] = -0.04;
  double theta1 = 0;//M_PI/4.0;	
  Eigen::AngleAxis<double> bb(theta1, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left	
  Eigen::Quaternion<double> quat_init; quat_init =  bb;
  q_cart[3] = quat_init.x();// 0.0;	
  q_cart[4] = quat_init.y(); //0.0;
  q_cart[5] = quat_init.z(); //sin(theta/2.0);
  q_cart[6] = quat_init.w(); //cos(theta/2.0);


  // Perform initial forward kinematics
  pinocchio::forwardKinematics(robot_model, *robot_data, q_robot);
  // Compute Joint Jacobians
  pinocchio::computeJointJacobians(robot_model,*robot_data, q_robot);
  // Update Frame Placements
  pinocchio::updateFramePlacements(robot_model, *robot_data); 
  // Perform initial forward kinematics
  pinocchio::forwardKinematics(cart_model, *cart_data, q_cart);
  // Compute Joint Jacobians
  pinocchio::computeJointJacobians(cart_model,*cart_data, q_cart);
  // Update Frame Placements
  pinocchio::updateFramePlacements(cart_model, *cart_data); 


  // Append the object onto the robot, and fill appended RobotModel
  pinocchio::appendModel(robot_model, cart_model, robot_geomModel, cart_geomModel, 0, pinocchio::SE3::Identity(), appended_model, appended_geomModel);

  appended_data = std::unique_ptr<pinocchio::Data>(new pinocchio::Data(appended_model));
  appended_geomData = std::unique_ptr<pinocchio::GeometryData>(new pinocchio::GeometryData(appended_geomModel));

  Eigen::VectorXd q_appended;
  q_appended = Eigen::VectorXd::Zero(appended_model.nq);

  for(int i=0; i<cart_model.nq; ++i){
  	q_appended[i] = q_cart[i];
  }
  int i = cart_model.nq;
  for(int j=0; j<robot_model.nq; ++j){
  	q_appended[i] = q_robot[j];
  	++i;
  }

  // Perform initial forward kinematics
  pinocchio::forwardKinematics(appended_model, *appended_data, q_appended);
  // Compute Joint Jacobians
  pinocchio::computeJointJacobians(appended_model,*appended_data, q_appended);
  // Update Frame Placements
  pinocchio::updateFramePlacements(appended_model, *appended_data); 
  // Update geometry Placements
  pinocchio::updateGeometryPlacements(appended_model, *appended_data, appended_geomModel, *appended_geomData, q_appended);

  std::cout << "appended_model: \n" << appended_model << std::endl;
  std::cout << "appended_geomModel: \n" << appended_geomModel << std::endl;
  // List Operational Space Frames
  for (int k=0 ; k<appended_model.frames.size() ; ++k){
    std::cout << "frame:" << k << " " << appended_model.frames[k].name << " : " << appended_data->oMf[k].translation().transpose() << std::endl;
  }
}