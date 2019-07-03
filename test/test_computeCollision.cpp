#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Multibody
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
// Parsers
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
// Algorithms
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
// Standard
#include <math.h> 
#include <iostream>
#include <iomanip>
#include <vector>
#include <boost/shared_ptr.hpp>

int main(int argc, char ** argv){
  std::string filename = (argc<=1) ? THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf" : argv[1];
  //const std::string srdf_filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/srdf/romeo.srdf";
  std::vector < std::string > packageDirs;
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";
  packageDirs.push_back(meshDir);

  // build URDF model
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);

  // build Geometry model
  pinocchio::GeometryModel geomModel;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geomModel, packageDirs );


  geomModel.addAllCollisionPairs();

  pinocchio::Data data(model);
  pinocchio::GeometryData geomData(geomModel);
  pinocchio::fcl::CollisionResult result;

  // Initialize Configuration -----------------------
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq); // Config 
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv); // Config Velocity
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv); // Config Acceleration

  // floating base joints: x, y, z
  q[0] = 0.0;  q[1] = 0.0;  q[2] = 0.0;

  // floating base quaternion: qx, qy, qz, qw
  double theta = M_PI/4.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left
  Eigen::Quaternion<double> quat_init; quat_init =  aa;

  q[3] = quat_init.x();// 0.0;
  q[4] = quat_init.y(); //0.0;
  q[5] = quat_init.z(); //sin(theta/2.0);
  q[6] = quat_init.w(); //cos(theta/2.0);

  // std::cout << "Initial " << q.transpose() << std::endl;
  // std::cout << "q = " << q.transpose() << std::endl;

  pinocchio::PairIndex PairId;
  int i;
  int j;
  

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);
  pinocchio::Index idx = geomModel.findCollisionPair(pinocchio::CollisionPair(1,10));


  // this computes a collision between a single collision pair and the result is stored in the collisionResults vector
  //pinocchio::computeCollision(geomModel,geomData,idx);

  // alternatively, there is the function computeCollisions
  	// Compute forward kinematics, updates geom placements, and calls computeCollision for every active pairs in GeometryData
  	// rtype is bool 
    // computeCollision which checks for each collision pair
      //computeCollision calls the function collide which returns if there is a collision
    // collisionResult struct: https://github.com/flexible-collision-library/fcl/blob/9dba579158109c0164bfe0e8b4a75c16bfc788f6/include/fcl/narrowphase/collision_result.h
  pinocchio::computeCollisions(model,data,geomModel,geomData,q);

  std::cout << "------------Collision Results: " << std::endl;
  for(j=0; j<geomData.collisionResults.size(); j++)
  {
    // computeCollisions sets collisionPairIndex to the first colliding pair
      PairId = geomData.collisionPairIndex;
      std::cout << "PairIndex PairId: " << PairId << std::endl;
    //We iterate through the collisionResults vector so we can determine if each pair is in collision
      result = geomData.collisionResults[j];    
      std::vector<pinocchio::fcl::Contact> contacts;
      result.getContacts(contacts);
      std::cout << contacts.size() << " contacts found" << std::endl;
      for(i=0; i<contacts.size(); i++)
      {
        std::cout << "position: " << contacts[i].pos << std::endl;
      }
  }
}
