#include <avatar_locomanipulation/data_types/footstep.hpp>

Footstep::Footstep(){
	position.setZero();
	orientation.setIdentity();
  robot_side = LEFT_FOOTSTEP;
  common_initialization();  
}

Footstep::Footstep(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in){
  position = pos_in;
  orientation = quat_in;
  robot_side = robot_side_in;
  common_initialization();  
}

Footstep::~Footstep(){	
}

void Footstep::common_initialization(){
	R_ori = orientation.toRotationMatrix();	
  std::cout << "[Footstep] Data Object constructed" << std::endl;
}

void Footstep::printInfo(){
	std::cout << "pos: " << position[0] << ", " << position[1] << ", " << position[2] << std::endl;
  std::cout << "ori: " << orientation.x() << ", " 
                     << orientation.y() << ", " 
                     << orientation.z() << ", "
                     << orientation.w() << std::endl;
}
