// Standard
#include <math.h>       
#include <iostream>

#include<avatar_locomanipulation/models/valkyrie_model.hpp>

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


}