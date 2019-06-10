#include <avatar_locomanipulation/data_types/footstep.hpp>

Footstep::Footstep(){
	position.setZero();
	orientation.setIdentity();

	std::cout << "[Footstep] Data Object constructed" << std::endl;
}

Footstep::~Footstep(){	
}