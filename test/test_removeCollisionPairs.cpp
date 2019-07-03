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
#include "pinocchio/algorithm/model.hpp"
// Spatial
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
// Standard
#include <math.h> 
#include <iostream>
#include <iomanip>
#include <vector>
#include <boost/shared_ptr.hpp>


int main(int argc, char ** argv){
  std::string filename = (argc<=1) ? THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf" : argv[1];
  std::string srdf_filename = (argc<=1) ? THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf" : argv[1];
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
  // Removes all collision pairs as specified in the srdf_filename
  pinocchio::srdf::removeCollisionPairs(model, geomModel, srdf_filename, false);

  pinocchio::Data data(model);
  pinocchio::GeometryData geomData(geomModel);
  pinocchio::fcl::CollisionResult result;
  int i, j, k;
  std::vector<pinocchio::fcl::Contact> contacts;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq); // Config 

  // floating base joints: x, y, z
  q[0] = 0.0;  q[1] = 0.0;  q[2] = 0.0;

  // floating base quaternion: qx, qy, qz, qw
  double theta = 0; //M_PI/4.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left
  Eigen::Quaternion<double> quat_init; quat_init =  aa;

  q[3] = quat_init.x();// 0.0;
  q[4] = quat_init.y(); //0.0;
  q[5] = quat_init.z(); //sin(theta/2.0);
  q[6] = quat_init.w(); //cos(theta/2.0);

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);

  pinocchio::computeCollisions(model,data,geomModel,geomData,q);

  // Check: Remove the line removeCollisionPairs and see if this changes
  std::cout << "geomData.collisionResults.size(): " << geomData.collisionResults.size() << std::endl;

  for(j=0; j<geomData.collisionResults.size(); j++)
  {
  	result = geomData.collisionResults[j];
  	pinocchio::CollisionPair idx = geomModel.collisionPairs[j];
  	std::cout << geomModel.getGeometryName(idx.first) << std::endl;
  	result.getContacts(contacts);
      	if(contacts.size() != 0)
      	{
      		for(k=0; k<contacts.size(); k++)
      		{
      			std::cout << "Contact Found Between: " << geomModel.getGeometryName(idx.first) << " and " << geomModel.getGeometryName(idx.second) << std::endl;
        		std::cout << "position: " << contacts[k].pos << std::endl;
        		std::cout << "-------------------" << std::endl;
      		}
      	}
  }

}