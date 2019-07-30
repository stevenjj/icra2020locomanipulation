#include <avatar_locomanipulation/data_types/manipulation_function.hpp>


ManipulationFunction::ManipulationFunction(){}
ManipulationFunction::~ManipulationFunction(){}


ManipulationFunction::ManipulationFunction(std::string filename){
	setWaypointsFromYaml(filename);
}

void ManipulationFunction::setWaypointsFromYaml(std::string filename){
	waypoint_list_yaml_filename = filename;
	// Create interpolator from waypoint list
	f_s.reset(new SixDimVec(filename));
}

void ManipulationFunction::getPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out){
	f_s->getPose(s, pos_out, quat_out);
}

