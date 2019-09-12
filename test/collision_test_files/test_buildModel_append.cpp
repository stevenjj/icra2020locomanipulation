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
	pinocchio::Model robot_model, cart_model;
  pinocchio::GeometryModel robot_geomModel, cart_geomModel;
  // data and geomdata
  std::unique_ptr<pinocchio::Data> robot_data, cart_data;
  std::unique_ptr<pinocchio::GeometryData> robot_geomData, cart_geomData;

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

  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(filename);
  pinocchio::urdf::buildModel(urdfTree, robot_model);


  // build model and geomModel
  // pinocchio::urdf::buildModel(filename,robot_model);
 //  pinocchio::urdf::buildGeom(robot_model, filename, pinocchio::COLLISION, robot_geomModel, meshDir );

 //  // Add and remove collision pairs
 //  robot_geomModel.addAllCollisionPairs();
 //  pinocchio::srdf::removeCollisionPairs(robot_model, robot_geomModel, srdf_filename, false);
 //  // Define the data and geomData
 //  robot_data = std::unique_ptr<pinocchio::Data>(new pinocchio::Data(robot_model));
	// robot_geomData = std::unique_ptr<pinocchio::GeometryData>(new pinocchio::GeometryData(robot_geomModel));
}