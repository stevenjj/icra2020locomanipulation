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
	rh_waypoint_list_yaml_filename = filename;
	// Create interpolator from waypoint list
	f_rh_s.reset(new CubicInterpolationSixDimVec(filename));
	// Set default to right hand
	right_hand_waypoints_set = true;
}

void ManipulationFunction::setRightWaypointsFromYaml(std::string filename){
	setWaypointsFromYaml(filename);
}
void ManipulationFunction::setLeftWaypointsFromYaml(std::string filename){
	lh_waypoint_list_yaml_filename = filename;
	// Create interpolator from waypoint list
	f_lh_s.reset(new CubicInterpolationSixDimVec(filename));
	left_hand_waypoints_set = true;
}

int ManipulationFunction::getManipulationType(){
	return manipulation_type;
}

void ManipulationFunction::setWorldTransform(const Eigen::Vector3d & translation, const Eigen::Quaterniond & orientation){
	transform_translation = translation;
	transform_orientation = orientation;
}

void ManipulationFunction::getPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out){
	f_rh_s->getPose(s, rhand_local_pos, rhand_local_ori);

	// Transform to world coordinates
	pos_out = transform_orientation.toRotationMatrix()*rhand_local_pos + transform_translation;
	quat_out = transform_orientation*rhand_local_ori;
}

void ManipulationFunction::getRightHandPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out){
	getPose(s, pos_out, quat_out);
}

void ManipulationFunction::getLeftHandPose(double & s, Eigen::Vector3d & pos_out, Eigen::Quaterniond & quat_out){
	f_lh_s->getPose(s, rhand_local_pos, rhand_local_ori);

	// Transform to world coordinates
	pos_out = transform_orientation.toRotationMatrix()*rhand_local_pos + transform_translation;
	quat_out = transform_orientation*rhand_local_ori;
}

void ManipulationFunction::setManipulationType(int type){
	manipulation_type = type;
}

void ManipulationFunction::common_initialization(){
	manipulation_type = MANIPULATE_TYPE_RIGHT_HAND;
	right_hand_waypoints_set = false;
	left_hand_waypoints_set = false;
	
	rhand_local_pos.setZero();
	rhand_local_ori.setIdentity();

	transform_translation.setZero();
	transform_orientation.setIdentity();
}