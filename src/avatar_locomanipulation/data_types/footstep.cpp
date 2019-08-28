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

void Footstep::setPosOriSide(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in){
  position = pos_in;
  orientation = quat_in;  
  robot_side = robot_side_in;
  R_ori = orientation.toRotationMatrix(); 
  updateContactLocations();
}

void Footstep::setPosOri(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in){
  position = pos_in;
  orientation = quat_in;
  R_ori = orientation.toRotationMatrix(); 
  updateContactLocations();
}

void Footstep::setRightSide(){  
  robot_side = RIGHT_FOOTSTEP;
}
void Footstep::setLeftSide(){  
  robot_side = LEFT_FOOTSTEP;
}

void Footstep::setMidFoot(){  
  robot_side = MID_FOOTSTEP;
}

void Footstep::common_initialization(){
	R_ori = orientation.toRotationMatrix();	

  // Initialize contact locations list
  local_contact_point_list.reserve(4);
  global_contact_point_list.reserve(4);
  for(int i = 0; i < 4; i++){
    local_contact_point_list.push_back(Eigen::Vector3d::Zero(3));
    global_contact_point_list.push_back(Eigen::Vector3d::Zero(3));
  }

  // Set Local Frame Contact Point List
  local_contact_point_list[0] = Eigen::Vector3d(sole_length/2.0, sole_width/2.0, 0.0);
  local_contact_point_list[1] = Eigen::Vector3d(sole_length/2.0, -sole_width/2.0, 0.0);
  local_contact_point_list[2] = Eigen::Vector3d(-sole_length/2.0, sole_width/2.0, 0.0);
  local_contact_point_list[3] = Eigen::Vector3d(-sole_length/2.0, -sole_width/2.0, 0.0);
  // Set Global Frame Contact Point List
  updateContactLocations();

  // std::cout << "[Footstep] Data Object constructed" << std::endl;
  // std::cout << "[Footstep] Size of local contact point list = " << local_contact_point_list.size() << std::endl;
  // std::cout << "[Footstep] Size of global_contact_point_list = " << local_contact_point_list.size() << std::endl;


}

void Footstep::updateContactLocations(){
  // Update global location of contact points based on the pose and orientation of the foot
  for(int i = 0; i < global_contact_point_list.size(); i++){
    global_contact_point_list[i] = R_ori*local_contact_point_list[i] + position;
  }
}


void Footstep::printInfo(){
  if ((robot_side == LEFT_FOOTSTEP) || (robot_side == RIGHT_FOOTSTEP)){
    std::cout << "side = " << (robot_side == LEFT_FOOTSTEP ? "LEFT_FOOTSTEP" : "RIGHT_FOOTSTEP") << std::endl;    
  }else if (robot_side == MID_FOOTSTEP){
    std::cout << "MID_FOOTSTEP" << std::endl;    
  }

	std::cout << "pos: " << position[0] << ", " << position[1] << ", " << position[2] << std::endl;
  std::cout << "ori: " << orientation.x() << ", " 
                     << orientation.y() << ", " 
                     << orientation.z() << ", "
                     << orientation.w() << std::endl;
}

void Footstep::computeMidfeet(const Footstep & footstep1, const Footstep & footstep2, Footstep & midfeet){
  midfeet.position = 0.5*(footstep1.position + footstep2.position);  
  midfeet.orientation = footstep1.orientation.slerp(0.5, footstep2.orientation);
  midfeet.R_ori = midfeet.orientation.toRotationMatrix(); 
  midfeet.robot_side = MID_FOOTSTEP;
}
