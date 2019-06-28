#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Multibody
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
// Algorithms
#include "pinocchio/algorithm/geometry.hpp"
// Parsers
#include "pinocchio/parsers/urdf.hpp"
// Spatial
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
    // allows us to define a more familiar translation and rotation and convert to SE3 as needed for GeometryObject
    // i.e:
    //// give the boxes defined transforms          ROTATION QUATERNION               TRANSLATION VECTOR
    // pinocchio::fcl::Transform3f tf1 (makeQuat (cos (pi/8), 0, 0, sin (pi/8)), pinocchio::fcl::Vec3f (-2, 1, .5));
    // SE3 placement = pinocchio::toPinocchioSE3(tf1)


// Standard
#include <math.h> 
#include <iostream>
#include <iomanip>
#include <vector>
#include <boost/shared_ptr.hpp>

using namespace pinocchio;

fcl::Quaternion3f makeQuat(double w, double x, double y, double z){
  pinocchio::fcl::Quaternion3f q;
  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  return q;
}

int main(int argc, char ** argv){
std::vector<std::string> getBodiesList();
  static double pi = M_PI;
  fcl::Transform3f tf1 (makeQuat (0, 0, 0, 0), pinocchio::fcl::Vec3f (1.5, 1.5, 1.5));
  SE3 placement = pinocchio::toPinocchioSE3(tf1);

// This script is a portion of: stack-of-tasks/pinocchio/unittest/geom.cpp
// this script creates a simple box 


  using namespace pinocchio;
  Model model;
  GeometryModel geomModel;

// define a JointIndex
  Model::JointIndex idx;
//--------------- FIRST
// addJoint is rtype JointIndexoint has two variations based on # of args
  	// This version has infinite bounds, the other is given finite bounds
  // it is fed:
	// 1 - index of parent joint
  	// 2 - the joint model -> these live in parsers/urdf/model.hxx
  	// 3 - joint placement = placement of the joint inside its parent joint
  	// 4 - name of the joint
  	// OPTIONALLY:
  	// 5, 6, 7, 8 - max_effort, max_velocity, min_config, max_config
  idx = model.addJoint(model.getJointId("universe"),JointModelPlanar(),SE3::Identity(),"planar1_joint");
// addJointFrame rtype is int, which is index of new frame; -1 returned if error
  // it is fed:
  	// 1 - index of the joint
  	// 2 - frame index, which is the index of parent frame. Default is -1
  		// when negative, the parent frame is the frame of the parent joint
  model.addJointFrame(idx);
// appendBodyToJoint rtype is void, but it appends a body to a given joint of a kinematic tree
  // it is fed:
  	// 1 - index of the supporting joint
  	// 2 - Y spatial inertia of body
  	// 3 - body_placement , relative placement of body regarding the parent joint
  		// default is identity
  model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity());
// addBodyFrame rtype int, index of the body frame, -1 if error
  // it is fed:
  	// 1 - name of body
  	// 2 - index of parent joint
  	// 3 - body_placement (see above)
  	// 4 - previousFrame , index of parent frame. Default is -1
  		// if negative, the parent frame is the frame of the parent joint
  model.addBodyFrame("planar1_body", idx, SE3::Identity());
//---------------- END FIRST

//---------------- SECOND  
  idx = model.addJoint(model.getJointId("universe"),JointModelPlanar(),SE3::Identity(),"planar2_joint");
  model.addJointFrame(idx);
  model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity());
  model.addBodyFrame("planar2_body", idx, SE3::Identity());
//---------------- END SECOND
  
  // fcl::Box is a data type defined in fcl, and we are giving sides 1x1x1
  // sample is the name of the object type boost::shared_ptr with an argument of type fcl::Box
  boost::shared_ptr<fcl::Box> sample(new fcl::Box(1, 1, 1));
  // we create object of frameindex and then get body1ID
  Model::FrameIndex body_id_1 = model.getBodyId("planar1_body");
  Model::JointIndex joint_parent_1 = model.frames[body_id_1].parent;
  // addGeometryObject rtype GeomIndex, which is ..
  // it adds a geometry object to a geometry model and the second argument sets a parent joint
  	//it has two versions which are fed:
  		// 1 - GeometryObject 
  			// GeomModel::GeometryObject, lives in pinocchio/src/multibody/fcl.hpp (INCLUDED WITH IFNDEF IN multibody/geometry.hpp)
        //GeometryObject(name, parentFrame, parentJoint, shared_ptr<fcl::CollisionGeometry>(collision object), placementSE3, meshPath, meshScale,...)
        // HERE: 
          // name = 'ff1_collision_object'
          // parentFrame = model.getBodyId("planar1_body")
          // parentJoint = joint_parent_1
          // shared_ptr<fcl::CollisionGeometry> = sample
          // placement = SE3::Identity()
  		// Optionally: 2 - corresponding model, which is used to assert the attributes of the object
  Model::JointIndex idx_geom1 = geomModel.addGeometryObject(GeometryObject("ff1_collision_object",
                                                                           model.getBodyId("planar1_body"),joint_parent_1,
                                                                           sample,placement, "", Eigen::Vector3d::Ones())
                                                            );
  // since the version of addGEometryObject is version 1, we must set a parent joint
  geomModel.geometryObjects[idx_geom1].parentJoint = model.frames[body_id_1].parent;
  
  
  boost::shared_ptr<fcl::Box> sample2(new fcl::Box(1, 1, 1));
  Model::FrameIndex body_id_2 = model.getBodyId("planar2_body");
  Model::JointIndex joint_parent_2 = model.frames[body_id_2].parent;
  // Note that this is the second version which sets the parent joint
  Model::JointIndex idx_geom2 = geomModel.addGeometryObject(GeometryObject("ff2_collision_object",
                                                                           model.getBodyId("planar2_body"),joint_parent_2,
                                                                           sample2,SE3::Identity(), "", Eigen::Vector3d::Ones()),
                                                            model);

// self explanatory. Defined in src/multibody/geometry.cpp
  // note that collision pairs b/w objects with same parent joint not added
  geomModel.addAllCollisionPairs();
  pinocchio::Data data(model);
  //geomData is where collision computations will be done
  pinocchio::GeometryData geomData(geomModel);

  int j, i;
  pinocchio::PairIndex PairId;
  pinocchio::fcl::CollisionResult result;
  pinocchio::fcl::DistanceResult dresult;

  //CollisionPair created collision pair from two collision object indexes, note index 1 < index 2, otherwise constructor will flip them

  std::cout << "------ Model ------ " << std::endl;
  std::cout << model;
  std::cout << "------ Geom ------ " << std::endl;
  std::cout << geomModel;
  std::cout << "------ DataGeom ------ " << std::endl;
  std::cout << geomData;

  Eigen::VectorXd q(model.nq);
  q <<  0, 0, 1, 0,
        0, 0, 1, 0 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);

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

  pinocchio::computeDistances(model,data,geomModel,geomData,q);

  std::cout << "------------Distance Query Results: " << std::endl;
  for(j=0; j<geomData.distanceResults.size(); j++)
  {
    //We iterate through the distanceResults vector so we can determine each pairs distance
      dresult = geomData.distanceResults[j];  
      // We can print the collision objects, but this is quite useless
        // Unsure how to go from collision object, which is an address to the name of the link it represents
      std::cout << "collision object 1: " << dresult.o1 << std::endl;
      std::cout << "collision object 2: " << dresult.o2 << std::endl;
      std::cout << "result.min_distance: " << dresult.min_distance << std::endl;
      //std::cout << "result.nearest_points[1]" << result.nearest_points[1] << std::endl;
  }


  q <<  2, 0, 1, 0,
        0, 0, 1, 0 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);

  std::cout << "------------Collision Results2: " << std::endl;
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
  pinocchio::computeDistances(model,data,geomModel,geomData,q);

  std::cout << "------------Distance Query Results2: " << std::endl;
  for(j=0; j<geomData.distanceResults.size(); j++)
  {
    //We iterate through the distanceResults vector so we can determine each pairs distance
      dresult = geomData.distanceResults[j];  
      // We can print the collision objects, but this is quite useless
        // Unsure how to go from collision object, which is an address to the name of the link it represents
      std::cout << "collision object 1: " << dresult.o1 << std::endl;
      std::cout << "collision object 2: " << dresult.o2 << std::endl;
      std::cout << "result.min_distance: " << dresult.min_distance << std::endl;
      //std::cout << "result.nearest_points[1]" << result.nearest_points[1] << std::endl;
  }

  q <<  0.99, 0, 1, 0,
        0, 0, 1, 0 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);

  std::cout << "------------Collision Results:3 " << std::endl;
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
  pinocchio::computeDistances(model,data,geomModel,geomData,q);

  std::cout << "------------Distance Query Results3: " << std::endl;
  for(j=0; j<geomData.distanceResults.size(); j++)
  {
    //We iterate through the distanceResults vector so we can determine each pairs distance
      dresult = geomData.distanceResults[j];  
      // We can print the collision objects, but this is quite useless
        // Unsure how to go from collision object, which is an address to the name of the link it represents
      std::cout << "collision object 1: " << dresult.o1 << std::endl;
      std::cout << "collision object 2: " << dresult.o2 << std::endl;
      std::cout << "result.min_distance: " << dresult.min_distance << std::endl;
      //std::cout << "result.nearest_points[1]" << result.nearest_points[1] << std::endl;
  }

  q <<  1.01, 0, 1, 0,
        0, 0, 1, 0 ;

  pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);

  std::cout << "------------Collision Results4: " << std::endl;
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
  pinocchio::computeDistances(model,data,geomModel,geomData,q);

  std::cout << "------------Distance Query Results4: " << std::endl;
  for(j=0; j<geomData.distanceResults.size(); j++)
  {
    //We iterate through the distanceResults vector so we can determine each pairs distance
      dresult = geomData.distanceResults[j];  
      // We can print the collision objects, but this is quite useless
        // Unsure how to go from collision object, which is an address to the name of the link it represents
      std::cout << "collision object 1: " << dresult.o1 << std::endl;
      std::cout << "collision object 2: " << dresult.o2 << std::endl;
      std::cout << "result.min_distance: " << dresult.min_distance << std::endl;
      //std::cout << "result.nearest_points[1]" << result.nearest_points[1] << std::endl;
  }

}