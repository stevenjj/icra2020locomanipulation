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

  pinocchio::GeometryModel::GeomIndex box_geom_id = geomModel.addGeometryObject(pinocchio::GeometryObject("box",
                                                                           model.getBodyId("planar1_body"),box_parent,
                                                                           sample,placement, "", Eigen::Vector3d::Ones())
                                                            );
  geomModel.geometryObjects[box_geom_id].parentJoint = model.frames[box_id].parent;

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

  int j, i, k;
  pinocchio::fcl::CollisionResult result;
  pinocchio::fcl::DistanceResult dresult;
  std::vector<pinocchio::fcl::Contact> contacts;

  pinocchio::computeCollisions(model,data,geomModel,geomData,p);
  pinocchio::computeDistances(model,data,geomModel,geomData,p);

  for(j=0; j<geomData.collisionResults.size(); j++)
  {
  	result = geomData.collisionResults[j];
  	dresult = geomData.distanceResults[j];
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
      	else
      	{

      		std::cout << "Minimum Distance Between: " << geomModel.getGeometryName(idx.first) << " and " << geomModel.getGeometryName(idx.second) << " = " << dresult.min_distance << std::endl;
      	}
  }

//   for(i=0; i<geomModel.ngeoms-1; i++)
//   {
//   	for(j=i+1; j<geomModel.ngeoms; j++)
//   	{
//   		geomModel.addCollisionPair(pinocchio::CollisionPair(i,j));
//   		pinocchio::Index idx = geomModel.findCollisionPair(pinocchio::CollisionPair(i,j));
// // --------- Collision Detection
//   		pinocchio::computeCollision(geomModel, geomData, idx);
//   		result = geomData.collisionResults[idx];
//       	result.getContacts(contacts);
//       	if(contacts.size() != 0)
//       	{
//       		for(k=0; k<contacts.size(); k++)
//       		{
//       			std::cout << "Contact Found Between: " << geomModel.getGeometryName(i) << " and " << geomModel.getGeometryName(j) << std::endl;
//         		std::cout << "position: " << contacts[k].pos << std::endl;
//         		std::cout << "-------------------" << std::endl;
//       		}
//       	}
// // --------- End Collision Detection
// // If no collision, compute distance
//       	else
//       	{
//       		pinocchio::computeDistance(geomModel, geomData, idx);
//       		dresult = geomData.distanceResults[idx];
//       		std::cout << "Minimum Distance Between: " << geomModel.getGeometryName(i) << " and " << geomModel.getGeometryName(j) << " = " << dresult.min_distance << std::endl;
//       	}
//   	}
//   }

  // std::cout << "box_geom_id: " << box_geom_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex pelvis_id = geomModel.getGeometryId("pelvis_0");
  // std::cout << "pelvis_id: " << pelvis_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftHipYawLink_id = geomModel.getGeometryId("leftHipYawLink_0");
  // std::cout << "leftHipYawLink_id: " << leftHipYawLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftHipRollLink_id = geomModel.getGeometryId("leftHipRollLink_0");
  // std::cout << "leftHipRollLink_id: " << leftHipRollLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftHipPitchLink_id = geomModel.getGeometryId("leftHipPitchLink_0");
  // std::cout << "leftHipPitchLink_id: " << leftHipPitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftKneePitchLink_id = geomModel.getGeometryId("leftKneePitchLink_0");
  // std::cout << "leftKneePitchLink_id: " << leftKneePitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftAnklePitchLink_id = geomModel.getGeometryId("leftAnklePitchLink_0");
  // std::cout << "leftAnklePitchLink_id: " << leftAnklePitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftFoot_id = geomModel.getGeometryId("leftFoot_0");
  // std::cout << "leftFoot_id: " << leftFoot_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightHipYawLink_id = geomModel.getGeometryId("rightHipYawLink_0");
  // std::cout << "rightHipYawLink_id: " << rightHipYawLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightHipRollLink_id = geomModel.getGeometryId("rightHipRollLink_0");
  // std::cout << "rightHipRollLink_id: " << rightHipRollLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightHipPitchLink_id = geomModel.getGeometryId("rightHipPitchLink_0");
  // std::cout << "rightHipPitchLink_id: " << rightHipPitchLink_id << std::endl;
  
  // pinocchio::GeometryModel::GeomIndex rightKneePitchLink_id = geomModel.getGeometryId("rightKneePitchLink_0");
  // std::cout << "rightKneePitchLink_id: " << rightKneePitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightAnklePitchLink_id = geomModel.getGeometryId("rightAnklePitchLink_0");
  // std::cout << "rightAnklePitchLink_id: " << rightAnklePitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightFoot_id = geomModel.getGeometryId("rightFoot_0");
  // std::cout << "rightFoot_id: " << rightFoot_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex torsoYawLink_id = geomModel.getGeometryId("torsoYawLink_0");
  // std::cout << "torsoYawLink_id: " << torsoYawLink_id << std::endl;
  
  // pinocchio::GeometryModel::GeomIndex torsoPitchLink_id = geomModel.getGeometryId("torsoPitchLink_0");
  // std::cout << "torsoPitchLink_id: " << torsoPitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex torso_id = geomModel.getGeometryId("torso_0");
  // std::cout << "torso_id: " << torso_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftShoulderPitchLink_id = geomModel.getGeometryId("leftShoulderPitchLink_0");
  // std::cout << "leftShoulderPitchLink_id: " << leftShoulderPitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftShoulderRollLink_id = geomModel.getGeometryId("leftShoulderRollLink_0");
  // std::cout << "leftShoulderRollLink_id: " << leftShoulderRollLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftElbowPitchLink_id = geomModel.getGeometryId("leftElbowPitchLink_0");
  // std::cout << "leftElbowPitchLink_id: " << leftElbowPitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftForearmLink_id = geomModel.getGeometryId("leftForearmLink_0");
  // std::cout << "leftForearmLink_id: " << leftForearmLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftWristRollLink_id = geomModel.getGeometryId("leftWristRollLink_0");
  // std::cout << "leftWristRollLink_id: " << leftWristRollLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftPalm_id = geomModel.getGeometryId("leftPalm_0");
  // std::cout << "leftPalm_id: " << leftPalm_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftIndexFingerPitch1Link_id = geomModel.getGeometryId("leftIndexFingerPitch1Link_0");
  // std::cout << "leftIndexFingerPitch1Link_id: " << leftIndexFingerPitch1Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftIndexFingerPitch2Link_id = geomModel.getGeometryId("leftIndexFingerPitch2Link_0");
  // std::cout << "leftIndexFingerPitch2Link_id: " << leftIndexFingerPitch2Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftIndexFingerPitch3Link_id = geomModel.getGeometryId("leftIndexFingerPitch3Link_0");
  // std::cout << "leftIndexFingerPitch3Link_id: " << leftIndexFingerPitch3Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftMiddleFingerPitch1Link_id = geomModel.getGeometryId("leftMiddleFingerPitch1Link_0");
  // std::cout << "leftMiddleFingerPitch1Link_id: " << leftMiddleFingerPitch1Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftMiddleFingerPitch2Link_id = geomModel.getGeometryId("leftMiddleFingerPitch2Link_0");
  // std::cout << "leftMiddleFingerPitch2Link_id: " << leftMiddleFingerPitch2Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftMiddleFingerPitch3Link_id = geomModel.getGeometryId("leftMiddleFingerPitch3Link_0");
  // std::cout << "leftMiddleFingerPitch3Link_id: " << leftMiddleFingerPitch3Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftPinkyPitch1Link_id = geomModel.getGeometryId("leftPinkyPitch1Link_0");
  // std::cout << "leftPinkyPitch1Link_id: " << leftPinkyPitch1Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftPinkyPitch2Link_id = geomModel.getGeometryId("leftPinkyPitch2Link_0");
  // std::cout << "leftPinkyPitch2Link_id: " << leftPinkyPitch2Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftPinkyPitch3Link_id = geomModel.getGeometryId("leftPinkyPitch3Link_0");
  // std::cout << "leftPinkyPitch3Link_id: " << leftPinkyPitch3Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftThumbRollLink_id = geomModel.getGeometryId("leftThumbRollLink_0");
  // std::cout << "leftThumbRollLink_id: " << leftThumbRollLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftThumbPitch1Link_id = geomModel.getGeometryId("leftThumbPitch1Link_0");
  // std::cout << "leftThumbPitch1Link_id: " << leftThumbPitch1Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftThumbPitch2Link_id = geomModel.getGeometryId("leftThumbPitch2Link_0");
  // std::cout << "leftThumbPitch2Link_id: " << leftThumbPitch2Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftThumbPitch3Link_id = geomModel.getGeometryId("leftThumbPitch3Link_0");
  // std::cout << "leftThumbPitch3Link_id: " << leftThumbPitch3Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex lowerNeckPitchLink_id = geomModel.getGeometryId("lowerNeckPitchLink_0");
  // std::cout << "lowerNeckPitchLink_id: " << lowerNeckPitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex neckYawLink_id = geomModel.getGeometryId("neckYawLink_0");
  // std::cout << "neckYawLink_id: " << neckYawLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex upperNeckPitchLink_id = geomModel.getGeometryId("upperNeckPitchLink_0");
  // std::cout << "upperNeckPitchLink_id: " << upperNeckPitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex head_id = geomModel.getGeometryId("head_0");
  // std::cout << "head_id: " << head_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex head_1_id = geomModel.getGeometryId("head_1");
  // std::cout << "head_1_id: " << head_1_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex hokuyo_link_id = geomModel.getGeometryId("hokuyo_link_0");
  // std::cout << "hokuyo_link_id: " << hokuyo_link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex hokuyo_link_1_id = geomModel.getGeometryId("hokuyo_link_1");
  // std::cout << "hokuyo_link_1_id: " << hokuyo_link_1_id << std::endl;
  
  // pinocchio::GeometryModel::GeomIndex rightShoulderPitchLink_id = geomModel.getGeometryId("rightShoulderPitchLink_0");
  // std::cout << "rightShoulderPitchLink_id: " << rightShoulderPitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightShoulderRollLink_id = geomModel.getGeometryId("rightShoulderRollLink_0");
  // std::cout << "rightShoulderRollLink_0: " << rightShoulderRollLink_0 << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightElbowPitchLink_id = geomModel.getGeometryId("rightElbowPitchLink_0");
  // std::cout << "rightElbowPitchLink_id: " << rightElbowPitchLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightForearmLink_id = geomModel.getGeometryId("rightForearmLink_0");
  // std::cout << "rightForearmLink_id: " << rightForearmLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex rightWristRollLink_id = geomModel.getGeometryId("rightWristRollLink_0");
  // std::cout << "rightWristRollLink_id: " << rightWristRollLink_id << std::endl;

  // //-/////////////////////////////////////////////////////////////////

  // pinocchio::GeometryModel::GeomIndex leftPalm_id = geomModel.getGeometryId("leftPalm_0");
  // std::cout << "leftPalm_id: " << leftPalm_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftIndexFingerPitch1Link_id = geomModel.getGeometryId("leftIndexFingerPitch1Link_0");
  // std::cout << "leftIndexFingerPitch1Link_id: " << leftIndexFingerPitch1Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftIndexFingerPitch2Link_id = geomModel.getGeometryId("leftIndexFingerPitch2Link_0");
  // std::cout << "leftIndexFingerPitch2Link_id: " << leftIndexFingerPitch2Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftIndexFingerPitch3Link_id = geomModel.getGeometryId("leftIndexFingerPitch3Link_0");
  // std::cout << "leftIndexFingerPitch3Link_id: " << leftIndexFingerPitch3Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftMiddleFingerPitch1Link_id = geomModel.getGeometryId("leftMiddleFingerPitch1Link_0");
  // std::cout << "leftMiddleFingerPitch1Link_id: " << leftMiddleFingerPitch1Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftMiddleFingerPitch2Link_id = geomModel.getGeometryId("leftMiddleFingerPitch2Link_0");
  // std::cout << "leftMiddleFingerPitch2Link_id: " << leftMiddleFingerPitch2Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftMiddleFingerPitch3Link_id = geomModel.getGeometryId("leftMiddleFingerPitch3Link_0");
  // std::cout << "leftMiddleFingerPitch3Link_id: " << leftMiddleFingerPitch3Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftPinkyPitch1Link_id = geomModel.getGeometryId("leftPinkyPitch1Link_0");
  // std::cout << "leftPinkyPitch1Link_id: " << leftPinkyPitch1Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftPinkyPitch2Link_id = geomModel.getGeometryId("leftPinkyPitch2Link_0");
  // std::cout << "leftPinkyPitch2Link_id: " << leftPinkyPitch2Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftPinkyPitch3Link_id = geomModel.getGeometryId("leftPinkyPitch3Link_0");
  // std::cout << "leftPinkyPitch3Link_id: " << leftPinkyPitch3Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftThumbRollLink_id = geomModel.getGeometryId("leftThumbRollLink_0");
  // std::cout << "leftThumbRollLink_id: " << leftThumbRollLink_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftThumbPitch1Link_id = geomModel.getGeometryId("leftThumbPitch1Link_0");
  // std::cout << "leftThumbPitch1Link_id: " << leftThumbPitch1Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftThumbPitch2Link_id = geomModel.getGeometryId("leftThumbPitch2Link_0");
  // std::cout << "leftThumbPitch2Link_id: " << leftThumbPitch2Link_id << std::endl;

  // pinocchio::GeometryModel::GeomIndex leftThumbPitch3Link_id = geomModel.getGeometryId("leftThumbPitch3Link_0");
  // std::cout << "leftThumbPitch3Link_id: " << leftThumbPitch3Link_id << std::endl;
}