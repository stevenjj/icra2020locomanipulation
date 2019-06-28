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
// Spatial
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
// Standard
#include <math.h> 
#include <iostream>
#include <iomanip>
#include <vector>
#include <boost/shared_ptr.hpp>

pinocchio::fcl::Quaternion3f makeQuat(double w, double x, double y, double z){
  pinocchio::fcl::Quaternion3f q;
  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  return q;
}

int main(int argc, char ** argv){

//------------------- Load Robot and give initial configuration
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

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq); // Config 
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv); // Config Velocity
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv); // Config Acceleration

  // floating base joints: x, y, z
  q[0] = 0.0;  q[1] = 0.0;  q[2] = 0.0;

  // floating base quaternion: qx, qy, qz, qw
  double theta = 0;//M_PI/4.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left
  Eigen::Quaternion<double> quat_init; quat_init =  aa;

  q[3] = quat_init.x();// 0.0;
  q[4] = quat_init.y(); //0.0;
  q[5] = quat_init.z(); //sin(theta/2.0);
  q[6] = quat_init.w(); //cos(theta/2.0);

//------------------- End Load Robot and give initial configuration

//----------------------------- Add Box to GeometryModel and give initial configuration
  static double pi = M_PI;
  pinocchio::fcl::Transform3f tf1 (makeQuat (0, 0, 1, 0), pinocchio::fcl::Vec3f (0, 0, 0));
  pinocchio::SE3 placement = pinocchio::toPinocchioSE3(tf1);

  pinocchio::Model::JointIndex idx;
  idx = model.addJoint(model.getJointId("universe"),pinocchio::JointModelPlanar(),pinocchio::SE3::Identity(),"planar1_joint");
  model.addJointFrame(idx);
  model.appendBodyToJoint(idx,pinocchio::Inertia::Random(),pinocchio::SE3::Identity());
  model.addBodyFrame("planar1_body", idx, pinocchio::SE3::Identity());

  boost::shared_ptr<pinocchio::fcl::Box> sample(new pinocchio::fcl::Box(1, 1, 1));
  pinocchio::Model::FrameIndex box_id = model.getBodyId("planar1_body");
  pinocchio::Model::JointIndex box_parent = model.frames[box_id].parent;

  pinocchio::Model::JointIndex box_geom_id = geomModel.addGeometryObject(pinocchio::GeometryObject("ff1_collision_object",
                                                                           model.getBodyId("planar1_body"),box_parent,
                                                                           sample,placement, "", Eigen::Vector3d::Ones())
                                                            );
  geomModel.geometryObjects[box_geom_index].parentJoint = model.frames[box_id].parent;

  Eigen::VectorXd p(model.nq);
  p <<  q,
  		0, 0, 1, 0;

  std::cout << "q.transpose" << q.transpose() << std::endl;
  std::cout << "p.transpose" << p.transpose() << std::endl;


//----------------------------- End Add Box to GeometryModel and give initial configuration
  geomModel.addAllCollisionPairs();

  pinocchio::Data data(model);
  pinocchio::GeometryData geomData(geomModel);
  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, p);

  int j, i;
  pinocchio::fcl::CollisionResult result;
  pinocchio::fcl::DistanceResult dresult;
  pinocchio::PairIndex PairId;


  pinocchio::computeCollisions(model,data,geomModel,geomData,p);

  std::cout << "------------Collision Results: " << std::endl;
  for(j=0; j<geomData.collisionResults.size(); j++)
  {
    // computeCollisions sets collisionPairIndex to the first colliding pair
      PairId = geomData.collisionPairIndex;
    //We iterate through the collisionResults vector so we can determine if each pair is in collision
      result = geomData.collisionResults[j];    
      std::vector<pinocchio::fcl::Contact> contacts;
      result.getContacts(contacts);
      //std::cout << contacts.size() << " contacts found" << std::endl;
      if(contacts.size() != 0)
      {
      	for(i=0; i<contacts.size(); i++)
      	{
      		std::cout << "PairIndex PairId: " << PairId << std::endl;
        	std::cout << "position: " << contacts[i].pos << std::endl;
        	std::cout << "-------------------" << std::endl;
      	}
      }
      
  }

//   pinocchio::computeDistances(model,data,geomModel,geomData,p);

//   std::cout << "------------Distance Query Results: " << std::endl;
//   for(j=0; j<geomData.distanceResults.size(); j++)
//   {
//     //We iterate through the distanceResults vector so we can determine each pairs distance
//       dresult = geomData.distanceResults[j];  
//       // We can print the collision objects, but this is quite useless
//         // Unsure how to go from collision object, which is an address to the name of the link it represents
//       std::cout << "collision object 1: " << dresult.o1 << std::endl;
//       std::cout << "collision object 2: " << dresult.o2 << std::endl;
//       std::cout << "result.min_distance: " << dresult.min_distance << std::endl;
//       //std::cout << "result.nearest_points[1]" << result.nearest_points[1] << std::endl;
//   }

}