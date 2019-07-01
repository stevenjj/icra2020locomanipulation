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
//NOTES:
//	it looks like to update the configuration vector, we have to start from the top
//		recreate geomValkyrie and geomBox and redefine p,q and then go thru the whole song and dance
//			i.e re appendGeometryModel
//	it seems like we still feed an unchanged q vector to computeCollisions and computeDistance,
//		but still it also checks collisions with the box

int main(int argc, char ** argv){

  pinocchio::Model valkyriemodel, boxmodel, valkyrieboxmodel;
  pinocchio::GeometryModel geomValkyrie, geomBox;

  //----------------- Robot Model
  std::string filename = (argc<=1) ? THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf" : argv[1];
  //const std::string srdf_filename = PINOCCHIO_SOURCE_DIR"/models/romeo/romeo_description/srdf/romeo.srdf";
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
  pinocchio::fcl::Transform3f tf1 (makeQuat (0, 0, 1, 0), pinocchio::fcl::Vec3f (0, 0, 0));
  pinocchio::SE3 placement = pinocchio::toPinocchioSE3(tf1);

  pinocchio::Model::JointIndex idx;
  idx = boxmodel.addJoint(boxmodel.getJointId("universe"),pinocchio::JointModelPlanar(),pinocchio::SE3::Identity(),"planar1_joint");
  boxmodel.addJointFrame(idx);
  boxmodel.appendBodyToJoint(idx,pinocchio::Inertia::Random(),pinocchio::SE3::Identity());
  boxmodel.addBodyFrame("planar1_body", idx, pinocchio::SE3::Identity());

  boost::shared_ptr<pinocchio::fcl::Box> sample(new pinocchio::fcl::Box(1, 1, 1));
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
  pinocchio::Data valkyriedata(valkyriemodel);
  pinocchio::GeometryData geomDataValkyrie(geomValkyrie);
  pinocchio::updateGeometryPlacements(valkyriemodel, valkyriedata, geomValkyrie, geomDataValkyrie, q);

  std::cout << "geomBox.ngeoms: " << geomBox.ngeoms << std::endl;
  std::cout << "geomValkyrie.ngeoms: " << geomValkyrie.ngeoms << std::endl;

  std::cout << "q.transpose: " << q.transpose() << std::endl;
  std::cout << "p.transpose: " << p.transpose() << std::endl;

  // Appends the second argument to the first (geomBox appended to geomValkyrie)
  // Note: the geomDataValkyrie corresponding to geomValkyrie will not be valid anymore, 
  	//and should be updated (or more simply, re-created from the new setting of geomValkyrie).
  pinocchio::appendGeometryModel(geomValkyrie, geomBox);

  std::cout << "geomBox.ngeoms: " << geomBox.ngeoms << std::endl;
  std::cout << "geomValkyrie.ngeoms: " << geomValkyrie.ngeoms << std::endl;

  std::cout << "q.transpose: " << q.transpose() << std::endl;
  std::cout << "p.transpose: " << p.transpose() << std::endl;

  Eigen::VectorXd r(valkyriemodel.nq);
  r << q;

  pinocchio::Data valkyrieboxdata(valkyriemodel);
  pinocchio::GeometryData geomDataValkyrieBox(geomValkyrie);
  pinocchio::updateGeometryPlacements(valkyriemodel, valkyrieboxdata, geomValkyrie, geomDataValkyrieBox, q);

  int j, i, k;
  pinocchio::fcl::CollisionResult result;
  pinocchio::fcl::DistanceResult dresult;
  std::vector<pinocchio::fcl::Contact> contacts;

  pinocchio::computeCollisions(valkyriemodel,valkyrieboxdata,geomValkyrie,geomDataValkyrieBox,r);

  for(j=0; j<geomDataValkyrieBox.collisionResults.size(); j++)
  {
  	result = geomDataValkyrieBox.collisionResults[j];
  	pinocchio::CollisionPair idx = geomValkyrie.collisionPairs[j];
  	// std::cout << geomValkyrie.getGeometryName(idx.first) << std::endl;
  	result.getContacts(contacts);
      	if(contacts.size() != 0)
      	{
      		for(k=0; k<contacts.size(); k++)
      		{
      			std::cout << "Contact Found Between: " << geomValkyrie.getGeometryName(idx.first) << " and " << geomValkyrie.getGeometryName(idx.second) << std::endl;
        		std::cout << "position: " << contacts[k].pos << std::endl;
        		std::cout << "-------------------" << std::endl;
      		}
      	}
      	else
      	{
      		pinocchio::computeDistance(geomValkyrie, geomDataValkyrieBox, geomValkyrie.findCollisionPair(idx));
      		dresult = geomDataValkyrieBox.distanceResults[j];
      		std::cout << "Minimum Distance Between: " << geomValkyrie.getGeometryName(idx.first) << " and " << geomValkyrie.getGeometryName(idx.second) << " = " << dresult.min_distance << std::endl;
      	}
  }


}