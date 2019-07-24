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

pinocchio::fcl::Quaternion3f makeQuat(double w, double x, double y, double z){
  pinocchio::fcl::Quaternion3f q;
  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  return q;
}


int main(int argc, char ** argv){

  pinocchio::Model valkyriemodel, boxmodel, valkyrieboxmodel;
  pinocchio::GeometryModel geomValkyrie, geomBox, geomValkyrieBox;

  //----------------- Robot Model
  std::string filename = (argc<=1) ? THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf" : argv[1];
  //const std::string srdf_filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/srdf/romeo.srdf";
  std::string srdf_filename = (argc<=1) ? THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf" : argv[1];
  std::vector < std::string > packageDirs;
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";
  packageDirs.push_back(meshDir);

  // build URDF model
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),valkyriemodel);

  // build Geometry model
  pinocchio::urdf::buildGeom(valkyriemodel, filename, pinocchio::COLLISION, geomValkyrie, packageDirs );

  Eigen::VectorXd q = Eigen::VectorXd::Zero(valkyriemodel.nq); // Config 

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
  //------------- End Robot Model

  //------------- Box Model
  static double pi = M_PI;
  pinocchio::fcl::Transform3f tf1 (makeQuat (0, 0, 1, 0), pinocchio::fcl::Vec3f (1.0, 0, 1.125));
  pinocchio::SE3 placement = pinocchio::toPinocchioSE3(tf1);

  pinocchio::Model::JointIndex idx;
  idx = boxmodel.addJoint(boxmodel.getJointId("universe"),pinocchio::JointModelPlanar(),pinocchio::SE3::Identity(),"planar1_joint");
  boxmodel.addJointFrame(idx);
  boxmodel.appendBodyToJoint(idx,pinocchio::Inertia::Random(),pinocchio::SE3::Identity());
  boxmodel.addBodyFrame("planar1_body", idx, pinocchio::SE3::Identity());

  boost::shared_ptr<pinocchio::fcl::Box> sample(new pinocchio::fcl::Box(0.05, 1.0, 2.25));
  pinocchio::Model::FrameIndex box_id = boxmodel.getBodyId("planar1_body");
  pinocchio::Model::JointIndex box_parent = boxmodel.frames[box_id].parent;

  pinocchio::GeometryModel::GeomIndex box_geom_id = geomBox.addGeometryObject(pinocchio::GeometryObject("box",
                                                                           boxmodel.getBodyId("planar1_body"),box_parent,
                                                                           sample,placement, "", Eigen::Vector3d::Ones())
                                                            );
  geomBox.geometryObjects[box_geom_id].parentJoint = boxmodel.frames[box_id].parent;

  Eigen::VectorXd p(boxmodel.nq);
  p << 0, 0, 1, 0;
  //------------- End Box Model

// Box:
  geomBox.addAllCollisionPairs();
  pinocchio::Data boxdata(boxmodel);
  pinocchio::GeometryData geomDataBox(geomBox);
  pinocchio::updateGeometryPlacements(boxmodel, boxdata, geomBox, geomDataBox, p);
// Valkyrie:
  geomValkyrie.addAllCollisionPairs();
  std::cout << "geomValkyrie.collisionPairs.size(): " << geomValkyrie.collisionPairs.size() << std::endl;
  //pinocchio::srdf::removeCollisionPairs(valkyriemodel, geomValkyrie, srdf_filename, false);
  std::cout << "geomValkyrie.collisionPairs.size(): " << geomValkyrie.collisionPairs.size() << std::endl;
  pinocchio::Data valkyriedata(valkyriemodel);
  pinocchio::GeometryData geomDataValkyrie(geomValkyrie);
  pinocchio::updateGeometryPlacements(valkyriemodel, valkyriedata, geomValkyrie, geomDataValkyrie, q);

//---------------- Begin Append
  std::cout << "geomBox.ngeoms: " << geomBox.ngeoms << std::endl;
  std::cout << "geomValkyrie.ngeoms: " << geomValkyrie.ngeoms << std::endl;

  std::cout << "q.transpose: " << q.transpose() << std::endl;
  std::cout << "p.transpose: " << p.transpose() << std::endl;

  // Append model: appends boxmodel to valkyrie model, and geomBox to geomValkyrie
  	// appends at some frame index fid with the relative config SE3::Identity()
  		// and it outputs to the final model and geomModel
  	// setting fid = valkyriemodel.frames.size() means that it is added to the last frame
  pinocchio::appendModel(valkyriemodel, boxmodel, geomValkyrie, geomBox, valkyriemodel.frames.size()-1, pinocchio::SE3::Identity(), valkyrieboxmodel, geomValkyrieBox);
  std::cout << "geomValkyrieBox.ngeoms: " << geomValkyrieBox.ngeoms << std::endl;
  std::cout << "boxmodel.nq: " << boxmodel.nq << std::endl;
  std::cout << "valkyriemodel.nq: " << valkyriemodel.nq << std::endl;
  std::cout << "valkyrieboxmodel.nq: " << valkyrieboxmodel.nq << std::endl;
//---------------- End Append

  // Create config vector for new appended model
  Eigen::VectorXd r(valkyrieboxmodel.nq);
  r << q, p;

  geomValkyrieBox.addAllCollisionPairs();
  pinocchio::Data valkyrieboxdata(valkyrieboxmodel);
  pinocchio::GeometryData geomDataValkyrieBox(geomValkyrieBox);
  pinocchio::updateGeometryPlacements(valkyrieboxmodel, valkyrieboxdata, geomValkyrieBox, geomDataValkyrieBox, r);

  // Define our counters, collision/distance results, contacts
  int j, i, k;
  pinocchio::fcl::CollisionResult result;
  pinocchio::fcl::DistanceResult dresult;
  std::vector<pinocchio::fcl::Contact> contacts;

  pinocchio::computeCollisions(valkyrieboxmodel, valkyrieboxdata, geomValkyrieBox, geomDataValkyrieBox, r);

  // Print relevant collision/distance data
  std::cout << "COLLISION RESULTS 1" << std::endl;
  for(j=0; j<geomDataValkyrieBox.collisionResults.size(); j++)
  {
  	result = geomDataValkyrieBox.collisionResults[j];
  	pinocchio::CollisionPair idx = geomValkyrieBox.collisionPairs[j];
  	result.getContacts(contacts);
      	if(contacts.size() != 0)
      	{
      		for(k=0; k<contacts.size(); k++)
      		{
      			std::cout << "Contact Found Between: " << geomValkyrieBox.getGeometryName(idx.first) << " and " << geomValkyrieBox.getGeometryName(idx.second) << std::endl;
        		std::cout << "position: " << contacts[k].pos << std::endl;
        		std::cout << "-------------------" << std::endl;
      		}
      	}
      	else
      	{
      		pinocchio::computeDistance(geomValkyrieBox, geomDataValkyrieBox, geomValkyrieBox.findCollisionPair(idx));
      		dresult = geomDataValkyrieBox.distanceResults[j];
      		std::cout << "Minimum Distance Between: " << geomValkyrieBox.getGeometryName(idx.first) << " and " << geomValkyrieBox.getGeometryName(idx.second) << " = " << dresult.min_distance << std::endl;
      	}
  }

  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;

  //------------------ Update the robot configuration 
  Eigen::VectorXd q1 = Eigen::VectorXd::Zero(valkyriemodel.nq); // Config 

  // floating base joints: x, y, z
  q1[0] = 0.0;  q1[1] = 0.0;  q1[2] = 0.0;

  // floating base quaternion: qx, qy, qz, qw
  double theta1 = M_PI/4.0;
  Eigen::AngleAxis<double> bb(theta1, Eigen::Vector3d(0.0, 0.0, 1.0)); // yaw pi/4 to the left
  Eigen::Quaternion<double> quat_init_1; quat_init_1 =  bb;

  q1[3] = quat_init_1.x();// 0.0;
  q1[4] = quat_init_1.y(); //0.0;
  q1[5] = quat_init_1.z(); //sin(theta/2.0);
  q1[6] = quat_init_1.w(); //cos(theta/2.0);
  //------------------ End Update the robot configuration 


  //------------------ Update the box configuration 
  Eigen::VectorXd p1(boxmodel.nq);
  p1 << 5, 0, 1, 0;
  //------------------ End Update the box configuration

  // Create config vector for new appended model
  Eigen::VectorXd r1(valkyrieboxmodel.nq);
  r1 << q1, p1;

  pinocchio::computeCollisions(valkyrieboxmodel, valkyrieboxdata, geomValkyrieBox, geomDataValkyrieBox, r1);

  // Print relevant collision/distance data
  std::cout << "COLLISION RESULTS 2" << std::endl;
  for(j=0; j<geomDataValkyrieBox.collisionResults.size(); j++)
  {
  	result = geomDataValkyrieBox.collisionResults[j];
  	pinocchio::CollisionPair idx = geomValkyrieBox.collisionPairs[j];
  	result.getContacts(contacts);
      	if(contacts.size() != 0)
      	{
      		for(k=0; k<contacts.size(); k++)
      		{
      			std::cout << "Contact Found Between: " << geomValkyrieBox.getGeometryName(idx.first) << " and " << geomValkyrieBox.getGeometryName(idx.second) << std::endl;
        		std::cout << "position: " << contacts[k].pos << std::endl;
        		std::cout << "-------------------" << std::endl;
      		}
      	}
      	else
      	{
      		pinocchio::computeDistance(geomValkyrieBox, geomDataValkyrieBox, geomValkyrieBox.findCollisionPair(idx));
      		dresult = geomDataValkyrieBox.distanceResults[j];
      		std::cout << "Minimum Distance Between: " << geomValkyrieBox.getGeometryName(idx.first) << " and " << geomValkyrieBox.getGeometryName(idx.second) << " = " << dresult.min_distance << std::endl;
      	}
  }

}