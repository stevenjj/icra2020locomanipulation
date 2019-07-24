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

pinocchio::fcl::Quaternion3f makeQuat(double w, double x, double y, double z){
  pinocchio::fcl::Quaternion3f q;
  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  return q;
}

int main(int argc, char ** argv){

  pinocchio::Model model;
  pinocchio::GeometryModel geomModel;

  pinocchio::Data data(model);
  pinocchio::GeometryData geomData(geomModel);
  pinocchio::fcl::DistanceResult result;

  //boost::shared_ptr <pinocchio::fcl::CollisionGeometry> CollisionGeometryPtr_t;

  // create two boxes 
  boost::shared_ptr <pinocchio::fcl::CollisionGeometry> s1 (new hpp::fcl::Box (1, 1, 1));
  boost::shared_ptr <pinocchio::fcl::CollisionGeometry> s2 (new hpp::fcl::Box (1, 1, 1));
  static double pi = M_PI;
  // give the boxes defined transforms
  pinocchio::fcl::Transform3f tf1 (makeQuat (cos (pi/8), 0, 0, sin (pi/8)), pinocchio::fcl::Vec3f (-2, 1, .5));
  pinocchio::fcl::Transform3f tf2 (makeQuat (cos (pi/8), 0, sin(pi/8),0), pinocchio::fcl::Vec3f (2, .5, .5));

  //pinocchio::fcl::CollisionObject;
  pinocchio::fcl::CollisionObject o1 (s1, tf1);
  pinocchio::fcl::CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  pinocchio::fcl::DistanceRequest distanceRequest (true, 0, 0, pinocchio::fcl::GST_INDEP);
  pinocchio::fcl::DistanceResult distanceResult;

  pinocchio::fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  const pinocchio::fcl::Vec3f& p1 = distanceResult.nearest_points [0];
  const pinocchio::fcl::Vec3f& p2 = distanceResult.nearest_points [1];


  std::cout << "Applied transformations on two boxes" << std::endl;
  std::cout << " Translation1 = " << tf1.getTranslation() << std::endl
	    << " Rotation1 = " << tf1.getRotation () << std::endl
	    << " Translation2 = " << tf2.getTranslation() << std::endl
	    << " Rotation2 = " << tf2.getRotation () << std::endl;
  std::cout << "Closest points(nearest_points): p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
        << ", distance(min_distance) = " << distanceResult.min_distance << std::endl;

}