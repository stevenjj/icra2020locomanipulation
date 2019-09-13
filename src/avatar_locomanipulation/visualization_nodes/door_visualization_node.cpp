#include <avatar_locomanipulation/visualization_nodes/door_visualization_node.hpp>


DoorVisualizationNode::DoorVisualizationNode(ros::NodeHandle* n_input, const std::string & parameter_file_input){
	s_offset = -0.1;
	s_current = s_offset;

	nh = n_input;
	parameter_file = parameter_file_input;
	std::cout << "DoorVisualizationNode Created\n";
	getParameters();
}


DoorVisualizationNode::~DoorVisualizationNode(){

}


void DoorVisualizationNode::s_callback(const std_msgs::Float64ConstPtr & msg){

	s_current = msg->data + s_offset;
}



void DoorVisualizationNode::getParameters(){
	ParamHandler param_handler;

	param_handler.load_yaml_file(THIS_PACKAGE_PATH + parameter_file);

	param_handler.getValue("radius", radius);

	param_handler.getValue("door_open_angle", door_open_angle);

	param_handler.getNestedValue({"hinge_fixed_frame_position", "x"}, hinge_frame_pos[0]);
	param_handler.getNestedValue({"hinge_fixed_frame_position", "y"}, hinge_frame_pos[1]);
	param_handler.getNestedValue({"hinge_fixed_frame_position", "z"}, hinge_frame_pos[2]);

	param_handler.getValue("pose_spacing", pose_spacing);

	double rx, ry, rz, rw;
	param_handler.getNestedValue({"hand_initial_orientation", "x"}, rx);
	param_handler.getNestedValue({"hand_initial_orientation", "y"}, ry);
	param_handler.getNestedValue({"hand_initial_orientation", "z"}, rz);
	param_handler.getNestedValue({"hand_initial_orientation", "w"}, rw);

	q_init.x() = rx; q_init.y() = ry; q_init.z() = rz; q_init.w() = rw;
	q_init.normalize();

	param_handler.getNestedValue({"hinge_fixed_frame_orientation", "x"}, rx);
	param_handler.getNestedValue({"hinge_fixed_frame_orientation", "y"}, ry);
	param_handler.getNestedValue({"hinge_fixed_frame_orientation", "z"}, rz);
	param_handler.getNestedValue({"hinge_fixed_frame_orientation", "w"}, rw);

	q_fixed.x() = rx; q_fixed.y() = ry; q_fixed.z() = rz; q_fixed.w() = rw;
	q_fixed.normalize();
}


void DoorVisualizationNode::getVizInformation(visualization_msgs::Marker & door_msg, tf::Transform & tf_world_hinge){

	Eigen::Quaternion<double> q, q_i, q_door_wrt_hinge;

	q_door_wrt_hinge.x() = 0.0; q_door_wrt_hinge.y() = 0.0;
	q_door_wrt_hinge.z() = 0.0; q_door_wrt_hinge.w() = 1.0;

	tf_world_hinge.setOrigin(tf::Vector3(hinge_frame_pos[0], hinge_frame_pos[1], hinge_frame_pos[2]));
	tf_world_hinge.setRotation(tf::Quaternion(q_fixed.x(), q_fixed.y(), q_fixed.z(), q_fixed.w()));

	q.x() = 0.0; q.y() = 0.0; q.z() = sin((s_current*door_open_angle) / 2); q.w() = cos((s_current*door_open_angle) / 2);

	q_i = (q * q_door_wrt_hinge);

	// Fill the Door marker msg
	door_msg.pose.position.x = (radius / 2.) * cos(1.57 + (s_current * door_open_angle));//sin(s_current * door_open_angle);;
	door_msg.pose.position.y = (radius / 2.) * sin(1.57 +  (s_current * door_open_angle));//cos(s_current * door_open_angle); 
	door_msg.pose.position.z = 0.0;
	door_msg.pose.orientation.x = q_i.x();
	door_msg.pose.orientation.y = q_i.y();
	door_msg.pose.orientation.z = q_i.z();
	door_msg.pose.orientation.w = q_i.w();
	door_msg.scale.x = 0.05; //handle_height*2;;
	door_msg.scale.y = radius; //0.05;
	door_msg.scale.z = hinge_frame_pos[2]*2; //radius;
	door_msg.header.frame_id = "hinge_frame";
	door_msg.text = "door";
	door_msg.color.r = 0.5f;
	door_msg.color.g = 0.5f;
	door_msg.color.b = 0.5f;
	door_msg.color.a = 0.8;
	door_msg.action = visualization_msgs::Marker::ADD;
	door_msg.type = visualization_msgs::Marker::CUBE;

}	