#include <avatar_locomanipulation/data_types/manipulation_function.hpp>


ManipulationFunction::ManipulationFunction(){
	common_initialization();	
}
ManipulationFunction::~ManipulationFunction(){}


ManipulationFunction::ManipulationFunction(std::string filename){
	setWaypointsFromYaml(filename);
	common_initialization();
}

void ManipulationFunction::setWaypointsFromYaml(std::string filename){
	waypoint_list_yaml_filename = filename;
	// Create interpolator from waypoint list
	f_s.reset(new CubicInterpolationSixDimVec(filename));
}


void ManipulationFunction::setWorldTransform(const Eigen::Vector3d & translation, const Eigen::Quaterniond & orientation){
	transform_translation = translation;
	transform_orientation = orientation;
}


void ManipulationFunction::getPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out){
	f_s->getPose(s, local_pos, local_ori);

	// Transform to world coordinates
	pos_out = transform_orientation.toRotationMatrix()*local_pos + transform_translation;
	quat_out = transform_orientation*local_ori;
}


void ManipulationFunction::common_initialization(){
	local_pos.setZero();
	local_ori.setIdentity();

	transform_translation.setZero();
	transform_orientation.setIdentity();
}