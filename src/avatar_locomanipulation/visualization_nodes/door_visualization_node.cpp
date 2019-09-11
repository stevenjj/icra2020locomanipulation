#include <avatar_locomanipulation/visualization_nodes/door_visualization_node.hpp>



DoorVisualizationNode::DoorVisualizationNode(){
	s_current = 0;
	std::cout << "DoorVisualizationNode Created\n";
	getParameters();
}


DoorVisualizationNode::DoorVisualizationNode(int & s_begin){
	s_current = s_begin;
	std::cout << "DoorVisualizationNode Created with initial s given\n";
	getParameters();
}


DoorVisualizationNode::~DoorVisualizationNode(){

}

void DoorVisualizationNode::getParameters(){
	ParamHandler param_handler;
	ParamHandler param_handler_2;

	param_handler.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/door_opening_parameters.yaml");

	param_handler_2.load_yaml_file(THIS_PACKAGE_PATH"door_open_trajectory.yaml");

	std::string wp1;
	wp1 = "waypoint_1";

	double x, y, z;
	param_handler_2.getNestedValue({wp1, "x"}, x);
	param_handler_2.getNestedValue({wp1, "y"}, y);
	param_handler_2.getNestedValue({wp1, "z"}, z);
	wp1_pos[0] = x; wp1_pos[1] = y; wp1_pos[2] = z;

	param_handler.getValue("radius", radius);

	param_handler.getValue("door_open_angle", door_open_angle);

	param_handler.getValue("handle_height", handle_height);

	param_handler.getValue("pose_spacing", pose_spacing);

	double rx, ry, rz, rw;
	param_handler.getNestedValue({"hand_initial_orientation", "x"}, rx);
	param_handler.getNestedValue({"hand_initial_orientation", "y"}, ry);
	param_handler.getNestedValue({"hand_initial_orientation", "z"}, rz);
	param_handler.getNestedValue({"hand_initial_orientation", "w"}, rw);

	q_init.x() = rx; q_init.y() = ry; q_init.z() = rz; q_init.w() = rw;
	q_init.normalize();
}


void DoorVisualizationNode::getVizInformation(visualization_msgs::Marker & door_msg, tf::Transform & tf_world_wp1, tf::Transform & tf_wp1_hinge){

	Eigen::Quaternion<double> q, q_i, q_fixed;

	tf_world_wp1.setOrigin(tf::Vector3(wp1_pos[0], wp1_pos[1], wp1_pos[2]));
	tf_world_wp1.setRotation(tf::Quaternion(0.5, -0.5, 0.5, 0.5));

	tf_wp1_hinge.setOrigin(tf::Vector3(0.0, 0.0, (0.9*radius)));
	tf_wp1_hinge.setRotation(tf::Quaternion(0, 0, 0, 1));

	// q_i.x() = 0.0; q_i.y() = 0.0; q_i.z() = sin((s_current*door_open_angle) / 2); q_i.w() = cos((s_current*door_open_angle) / 2);
	q_i.x() = sin((s_current*door_open_angle) / 2); q_i.y() = 0.0; q_i.z() = 0.0; q_i.w() = cos((s_current*door_open_angle) / 2);
	
	q_fixed.x() = 0.5; q_fixed.y() = -0.5; q_fixed.z() = 0.5; q_fixed.w() = 0.5;
	q = q_i * q_fixed;
	// Fill the Door marker msg
	door_msg.pose.position.x = 0.0;
	door_msg.pose.position.y = (radius / 2.) * sin(s_current * door_open_angle);
	door_msg.pose.position.z = -(radius / 2.) * cos(s_current * door_open_angle);
	door_msg.pose.orientation.x = q.x();
	door_msg.pose.orientation.y = q.y();
	door_msg.pose.orientation.z = q.z();
	door_msg.pose.orientation.w = q.w();
	door_msg.scale.x = radius; //handle_height*2;;
	door_msg.scale.y = handle_height*2; //0.05;
	door_msg.scale.z = 0.05; //radius;
	door_msg.header.frame_id = "hinge_frame";
	door_msg.text = "door";
	door_msg.color.r = 0.5f;
	door_msg.color.g = 0.5f;
	door_msg.color.b = 0.5f;
	door_msg.color.a = 0.8;
	door_msg.action = visualization_msgs::Marker::ADD;
	door_msg.type = visualization_msgs::Marker::CUBE;

}	