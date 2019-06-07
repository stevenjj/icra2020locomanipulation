// Standard
#include <math.h>       
#include <iostream>

#include <avatar_locomanipulation/models/valkyrie_model.hpp>

int main(int argc, char ** argv){
	std::cout << "Initialize Valkyrie Model" << std::endl;
	ValkyrieModel valkyrie;

	// Test Dimensions
	std::cout << "joint dimension: " << valkyrie.getDimQ() << std::endl;
	std::cout << "joint velocity dimension: " << valkyrie.getDimQdot() << std::endl;
	
	// Print out joint names and frames
	valkyrie.printJointNames();
	valkyrie.printFrameNames();

	// Test joint indexing
	std::string joint_sample = "leftHipYaw";
	std::cout << joint_sample << " id = " << valkyrie.getJointIndex(joint_sample) << ", expected: 7" << std::endl;

	// Test Kinematics
	Eigen::VectorXd q_start = Eigen::VectorXd::Zero(valkyrie.getDimQ());
	valkyrie.updateFullKinematics(q_start);

	Eigen::MatrixXd J_rpalm(6, valkyrie.getDimQdot()); J_rpalm.fill(0);
	valkyrie.get6DTaskJacobian("rightPalm", J_rpalm);

	std::cout << "6D Jacobian of the right palm:" << std::endl;
	std::cout << J_rpalm << std::endl;

	// Test Obtaining Frame pose
    Eigen::Vector3d rpalm_pos; rpalm_pos.setZero();
    Eigen::Quaternion<double> rpalm_ori; rpalm_ori.setIdentity();
    valkyrie.getFrameWorldPose("rightPalm", rpalm_pos, rpalm_ori);

    std::cout << "right palm translation w.r.t world:" << rpalm_pos.transpose() << std::endl;
    std::cout << "right palm orientation w.r.t world (x,y,z,w):" << rpalm_ori.x() << " " <<
    																rpalm_ori.y() << " " <<
    																rpalm_ori.z() << " " <<
    																rpalm_ori.w() << " " << std::endl;
	// Get Dynamics Matrices
    Eigen::VectorXd qdot_start(valkyrie.getDimQdot());
    qdot_start = Eigen::VectorXd::Ones(valkyrie.getDimQdot());
	
	valkyrie.computeInertiaMatrix(q_start);
	valkyrie.computeInertiaMatrixInverse(q_start);
	valkyrie.computeCoriolisMatrix(q_start, qdot_start);
	valkyrie.computeGravityVector(q_start);

	// std::cout << "Inertia Matrix" << std::endl;
	// std::cout << valkyrie.A << std::endl;

	// std::cout << "Inertia Matrix Inverse" << std::endl;
	// std::cout << valkyrie.Ainv << std::endl;

	// std::cout << "Coriolis Matrix:" << std::endl;
	// std::cout << valkyrie.C << std::endl;

	std::cout << "Gravity Vector:" << valkyrie.g.transpose() << std::endl;

	// Test Forward Integration
	Eigen::VectorXd q_post(valkyrie.getDimQ()); 
	q_post.setZero();
	qdot_start.setZero();

	const double dt = 1.0;
	qdot_start[0] = 0.5; // move forward with 0.5 m/s
	qdot_start[5] = M_PI/4.0; // yaw by pi/4 rad/s to the left
	valkyrie.forwardIntegrate(q_start, qdot_start*dt, q_post);

	std::cout << "Forward Integration:" << q_post.transpose() << std::endl; 

}