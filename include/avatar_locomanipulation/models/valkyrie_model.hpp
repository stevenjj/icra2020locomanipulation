#ifndef ALM_VALKYRIE_MODEL_H
#define ALM_VALKYRIE_MODEL_H

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

// Macros
#define VAL_MODEL_NUM_FLOATING_JOINTS 7 // 3 for x,y,z and 4 for qx, qy, qz, qw
#define VAL_MODEL_JOINT_INDX_OFFSET 2 //pinocchio attaches a universe joint and a root joint that we need to remove


class ValkyrieModel{
public:
	ValkyrieModel();
    ValkyrieModel(const std::string & filename);

	~ValkyrieModel();
	
    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;

    Eigen::MatrixXd A; // inertia matrix
    Eigen::MatrixXd Ainv; // inverse of the inertia matrix
    Eigen::MatrixXd C; // coriolis matrix
    Eigen::VectorXd g; // gravity vector


    /* updateFullKinematics
    Input: a vector of configuration with dimension model.nq to update the kinematics.   
    */
    void updateFullKinematics(const Eigen::VectorXd & q_update);

    /* get6DTaskJacobian
    Input: the frame name.   
    Output: the 6D task jacobian (dimension 6 x model.nv) expressed in world frame
            with the linear components first: [(dx/dq)^T , (dy/dq)^T, (dz/dq)^T]^T
            then the rotational components: [(wx/dq)^T , (wy/dq)^T, (wz/dq)^T]^T
    */
    void get6DTaskJacobian(const std::string & frame_name, Eigen::MatrixXd & J_out);

    /* getFrameWorldPose
    Input: the frame name.   
    Output: the position and orientation of the frame with respect to world.
    */

    void getFrameWorldPose(const std::string & name, Eigen::Vector3d & pos, Eigen::Quaternion<double> & ori);

    /* getJointIndex
    Input std::string name
    Output: The joint index in configuration space
        For Valkyrie, the first 7 joints are floating base joints. 3 for the linear and 4 for the rotation described as a quaternion.       
        Per C++ standard, the index starts at 0.

        Thus, for a "leftHipYaw" input, which is the first joint in the kinematic chain,
        this function should return 7.
    */
    int getJointIndex(const std::string & name); 

    /* getDimQ();
    Output: The dimension of the configuration space
    */
    int getDimQ();
    /* getDimQdot();
    Output: The dimension of the configuration space
    */
    int getDimQdot();

    // Tangent space forward integration given q_start, qdot*dt 
    void forwardIntegrate(const Eigen::VectorXd & q_start, const Eigen::VectorXd & qdotDt, Eigen::VectorXd & q_post);

    // Computes the inertia matrix dim(model.nv x model.nv) given configuration and stores the result.
    void computeInertiaMatrix(const Eigen::VectorXd & q);
    // Computes the inverse of the inertia matrix dim(model.nv x model.nv) given configuration and stores the result.
    void computeInertiaMatrixInverse(const Eigen::VectorXd & q);
    // Computes the coriolis matrix dim(model.nv x model.nv) given configuration and velocity and stores the result.
    void computeCoriolisMatrix(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot);
    // Computes the gravity vector dim(model.nv) given configuration and stores the result
    void computeGravityVector(const Eigen::VectorXd & q);

    void printJointNames();
    void printFrameNames();

private:
    // private temporary index value holders. Prevents memory allocation on runtime
    pinocchio::FrameIndex tmp_frame_index;
    pinocchio::JointIndex tmp_joint_index;
    void buildPinocchioModel(const std::string & filename);
    void commonInitialization();
};

#endif 