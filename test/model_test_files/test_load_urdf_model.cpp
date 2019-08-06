#include <Configuration.h> // Package Path
#include <avatar_locomanipulation/enable_pinocchio_with_hpp_fcl.h> // Enable HPP FCL
// Multibody
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
// Algorithms
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp" // Jacobian Frame Computation 
#include "pinocchio/algorithm/aba.hpp" // Articulated Body Algorithm
#include "pinocchio/algorithm/crba.hpp" // Composite Rigid Body Algorithm
#include "pinocchio/algorithm/rnea.hpp" // Recursive Newton-Euler Algorithm
// Parsers
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

// Utils
#include "pinocchio/utils/timer.hpp"

// Standard
#include <math.h>       

int main(int argc, char ** argv){
  PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);

  std::string filename = (argc<=1) ? THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf" : argv[1];
  std::vector < std::string > packageDirs;
  std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";
  packageDirs.push_back(meshDir);

  // Build URDF Model
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);

  // Build Geometry model to check for collisions
  pinocchio::GeometryModel geom;
  pinocchio::urdf::buildGeom(model, filename, pinocchio::COLLISION, geom, packageDirs);
  pinocchio::Data data(model);
  pinocchio::GeometryData geomData(geom);

  //<---------- Added By Ryan
  geom.addAllCollisionPairs();
  
  std::cout << "------ Model ------ " << std::endl;
  std::cout << model;
  std::cout << "------ Geom ------ " << std::endl;
  std::cout << geom;
  std::cout << "------ DataGeom ------ " << std::endl;
  std::cout << geomData;
  //<---------- End Added By Ryan

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

  std::cout << "Initial " << q.transpose() << std::endl;
  std::cout << "q = " << q.transpose() << std::endl;
  // End Initialize Configuration -----------------------

  // Test Kinematics Kinematics:
  // Note that this function does not check if the quaternion is a valid unit quaternion.
  timer.tic();
  pinocchio::forwardKinematics(model,data,q);
  std::cout << "Forward kinematics took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  // Compute Joint Jacobians
  std::cout << "  ";  timer.tic();
  pinocchio::computeJointJacobians(model,data, q);
  std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " jacobians computation time" << std::endl;
  // Update Frame Placements
  std::cout << "  ";  timer.tic();
  pinocchio::updateFramePlacements(model, data);    
  std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " frame placements computation time" << std::endl;

  // List Joint Names
  for (int k=0 ; k<model.njoints ; ++k) {
    std::cout << model.names[k] << "\t: "
              << data.oMi[k].translation().transpose() << std::endl;
  }

  // List Operational Space Frames
  for (int k=0 ; k<model.frames.size() ; ++k){
    std::cout << "frame:" << k << " " << model.frames[k].name << " : " << data.oMf[k].translation().transpose() << std::endl;
  }


  // Test Inverse Dynamics
  timer.tic();
  const Eigen::VectorXd & tau = pinocchio::rnea(model,data,q,v,a);
  std::cout << "tau = " << tau.transpose() << std::endl;
  std::cout << "Inverse dynamics took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;



  return 0;
}