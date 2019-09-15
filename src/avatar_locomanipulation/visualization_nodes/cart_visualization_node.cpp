#include <avatar_locomanipulation/visualization_nodes/cart_visualization_node.hpp>



CartVisualizationNode::CartVisualizationNode(ros::NodeHandle* n_input, const std::string & parameter_file_input){
	s_current = 0;
	nh = n_input;
	parameter_file = parameter_file_input;
	std::cout << "CartVisualizationNode Created\n";
	getParameters();
}


CartVisualizationNode::~CartVisualizationNode(){

}

void CartVisualizationNode::s_callback(const std_msgs::Float64ConstPtr & msg){

	s_current = msg->data;
}


void CartVisualizationNode::getParameters(){
	ParamHandler param_handler;

	param_handler.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/cart_parameters.yaml");

	param_handler.getValue("radius", radius);

	param_handler.getValue("rotation_angle", rotation_angle);

	param_handler.getValue("linear_length", linear_length);

	param_handler.getValue("pose_spacing", pose_spacing);

	param_handler.getValue("hand_distance", hand_distance);

	param_handler.getNestedValue({"fixed_frame_position", "x"}, fixed_frame_pos[0]);
	param_handler.getNestedValue({"fixed_frame_position", "y"}, fixed_frame_pos[1]);
	param_handler.getNestedValue({"fixed_frame_position", "z"}, fixed_frame_pos[2]);


	double rx, ry, rz, rw;

	param_handler.getNestedValue({"right_hand_initial_orientation", "x"}, rx);
	param_handler.getNestedValue({"right_hand_initial_orientation", "y"}, ry);
	param_handler.getNestedValue({"right_hand_initial_orientation", "z"}, rz);
	param_handler.getNestedValue({"right_hand_initial_orientation", "w"}, rw);

	q_r_init.x() = rx; q_r_init.y() = ry; q_r_init.z() = rz; q_r_init.w() = rw;
	q_r_init.normalize();

	param_handler.getNestedValue({"left_hand_initial_orientation", "x"}, rx);
	param_handler.getNestedValue({"left_hand_initial_orientation", "y"}, ry);
	param_handler.getNestedValue({"left_hand_initial_orientation", "z"}, rz);
	param_handler.getNestedValue({"left_hand_initial_orientation", "w"}, rw);

	q_l_init.x() = rx; q_l_init.y() = ry; q_l_init.z() = rz; q_l_init.w() = rw;
	q_l_init.normalize();

	param_handler.getNestedValue({"fixed_frame_orientation", "x"}, rx);
	param_handler.getNestedValue({"fixed_frame_orientation", "y"}, ry);
	param_handler.getNestedValue({"fixed_frame_orientation", "z"}, rz);
	param_handler.getNestedValue({"fixed_frame_orientation", "w"}, rw);

	q_fixed.x() = rx; q_fixed.y() = ry; q_fixed.z() = rz; q_fixed.w() = rw;
	q_fixed.normalize();
}


void CartVisualizationNode::getVizInformation(tf::Transform & tf_world_fixed, tf::Transform & tf_fixed_cart){

  // Define the fixed frame transform
  tf_world_fixed.setOrigin(tf::Vector3(fixed_frame_pos[0], fixed_frame_pos[1], fixed_frame_pos[2]));
  tf_world_fixed.setRotation(tf::Quaternion(q_fixed.x(), q_fixed.y(), q_fixed.z(), q_fixed.w()));

  // First need to determine the bounds of s for when we are lifting vs when we are turning
  double arc_length = rotation_angle * radius;

  N = static_cast<int>(arc_length / pose_spacing);// Number of wps to describe the arc
  M = static_cast<int>(linear_length / pose_spacing); // Number of wps to describe each of the up and down motions

  int total_wps = N + M + 1;

  Eigen::Quaternion<double> q, q_i; 

  double percent_pushing = (static_cast<double>(M) / static_cast<double>(total_wps));
  double percent_turning = (static_cast<double>(N) / static_cast<double>(total_wps));

  if(s_current < percent_pushing){
    tf_fixed_cart.setOrigin(tf::Vector3((s_current / percent_pushing) * linear_length, -(radius + (hand_distance/2)), 0.0));
    tf_fixed_cart.setRotation(tf::Quaternion(0., 0., 0., 1.));
  }
  else{
    double s_local;
    s_local = (s_current - percent_pushing) / (1. - percent_pushing);
    tf_fixed_cart.setOrigin(tf::Vector3((radius + linear_length + (hand_distance/2))*cos(-1.57 + (s_local*rotation_angle)), (radius + linear_length)*sin(-1.57 + (s_local*rotation_angle)), 0.0 ) );
    
    q.x() = 0.0; q.y() = 0.0; q.z() = sin((s_local * rotation_angle) / 2); q.w() = cos((s_local * rotation_angle) / 2);
    q_i = q * q_fixed;

    tf_fixed_cart.setRotation(tf::Quaternion(q_i.x(), q_i.y(), q_i.z(), q_i.w()));
  }
}	


void CartVisualizationNode::fillCartMarkerArray(visualization_msgs::MarkerArray & cart_msg){

	visualization_msgs::Marker temp;

  // First generate  handle of the cart
  temp.pose.position.x = 0.0;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = 0.0;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.9;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "handle_top";
  temp.ns = "ns";
  temp.id = 1;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.05;
  temp.pose.position.y = -0.42;
  temp.pose.position.z = -0.1;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = -0.25;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 0.96825;
  temp.scale.x = 0.03;
  temp.scale.y = 0.02;
  temp.scale.z = 0.2;
  temp.header.frame_id = "cart_frame";
  temp.text = "handle_side_1";
  temp.ns = "ns";
  temp.id = 2;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.05;
  temp.pose.position.y = 0.42;
  temp.pose.position.z = -0.1;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = -0.25;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 0.96825;
  temp.scale.x = 0.03;
  temp.scale.y = 0.02;
  temp.scale.z = 0.2;
  temp.header.frame_id = "cart_frame";
  temp.text = "handle_side_2";
  temp.ns = "ns";
  temp.id = 3;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.46;
  temp.pose.position.y = 0.35;
  temp.pose.position.z = -0.185;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = -0.1;
  temp.pose.orientation.w = 0.995;
  temp.scale.x = 0.75;
  temp.scale.y = 0.03;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_top_left";
  temp.ns = "ns";
  temp.id = 4;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.46;
  temp.pose.position.y = -0.35;
  temp.pose.position.z = -0.185;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.1;
  temp.pose.orientation.w = 0.995;
  temp.scale.x = 0.75;
  temp.scale.y = 0.03;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_top_right";
  temp.ns = "ns";
  temp.id = 5;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.81;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = -0.185;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.52;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_top_far";
  temp.ns = "ns";
  temp.id = 6;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.1;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = -0.185;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.84;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_top_close";
  temp.ns = "ns";
  temp.id = 7;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.46;
  temp.pose.position.y = 0.35;
  temp.pose.position.z = -0.385;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = -0.1;
  temp.pose.orientation.w = 0.995;
  temp.scale.x = 0.75;
  temp.scale.y = 0.03;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_mid_left";
  temp.ns = "ns";
  temp.id = 8;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.46;
  temp.pose.position.y = -0.35;
  temp.pose.position.z = -0.385;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.1;
  temp.pose.orientation.w = 0.995;
  temp.scale.x = 0.75;
  temp.scale.y = 0.03;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_mid_right";
  temp.ns = "ns";
  temp.id = 9;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.81;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = -0.385;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.52;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_mid_far";
  temp.ns = "ns";
  temp.id = 10;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.1;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = -0.385;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.84;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_mid_close";
  temp.ns = "ns";
  temp.id = 11;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);


  temp.pose.position.x = 0.82;
  temp.pose.position.y = 0.285;
  temp.pose.position.z = -0.285;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.03;
  temp.scale.z = 0.2;
  temp.header.frame_id = "cart_frame";
  temp.text = "cart_top_mid_far_left";
  temp.ns = "ns";
  temp.id = 12;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.82;
  temp.pose.position.y = -0.285;
  temp.pose.position.z = -0.285;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.03;
  temp.scale.z = 0.2;
  temp.header.frame_id = "cart_frame";
  temp.text = "cart_top_mid_far_right";
  temp.ns = "ns";
  temp.id = 13;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.1;
  temp.pose.position.y = 0.42;
  temp.pose.position.z = -0.285;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.03;
  temp.scale.z = 0.2;
  temp.header.frame_id = "cart_frame";
  temp.text = "cart_top_mid_close_left";
  temp.ns = "ns";
  temp.id = 14;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.1;
  temp.pose.position.y = -0.42;
  temp.pose.position.z = -0.285;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.03;
  temp.scale.z = 0.2;
  temp.header.frame_id = "cart_frame";
  temp.text = "cart_top_mid_close_right";
  temp.ns = "ns";
  temp.id = 15;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.46;
  temp.pose.position.y = 0.35;
  temp.pose.position.z = -fixed_frame_pos[2] + 0.08;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = -0.1;
  temp.pose.orientation.w = 0.995;
  temp.scale.x = 0.75;
  temp.scale.y = 0.03;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_lower_left";
  temp.ns = "ns";
  temp.id = 16;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.46;
  temp.pose.position.y = -0.35;
  temp.pose.position.z = -fixed_frame_pos[2] + 0.08;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.1;
  temp.pose.orientation.w = 0.995;
  temp.scale.x = 0.75;
  temp.scale.y = 0.03;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_lower_lower_right";
  temp.ns = "ns";
  temp.id = 17;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.81;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = -fixed_frame_pos[2] + 0.08;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.52;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_lower_far";
  temp.ns = "ns";
  temp.id = 18;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.1;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = -fixed_frame_pos[2] + 0.08;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.84;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "main_cart_lower_close";
  temp.ns = "ns";
  temp.id = 19;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);


  temp.pose.position.x = 0.1;
  temp.pose.position.y = 0.42;
  temp.pose.position.z = -0.5;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.03;
  temp.scale.z = 0.48;
  temp.header.frame_id = "cart_frame";
  temp.text = "cart_lower_mid_close_left";
  temp.ns = "ns";
  temp.id = 20;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);

  temp.pose.position.x = 0.1;
  temp.pose.position.y = -0.42;
  temp.pose.position.z = -0.5;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.03;
  temp.scale.y = 0.03;
  temp.scale.z = 0.48;
  temp.header.frame_id = "cart_frame";
  temp.text = "cart_lower_mid_close_right";
  temp.ns = "ns";
  temp.id = 21;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CUBE;

  cart_msg.markers.push_back(temp);


  temp.pose.position.x = 0.1;
  temp.pose.position.y = 0.42;
  temp.pose.position.z = -fixed_frame_pos[2] + 0.04;
  temp.pose.orientation.x = 0.707;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 0.707;
  temp.scale.x = 0.085;
  temp.scale.y = 0.085;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "wheel1";
  temp.ns = "ns";
  temp.id = 22;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CYLINDER;

  cart_msg.markers.push_back(temp);


  temp.pose.position.x = 0.1;
  temp.pose.position.y = -0.42;
  temp.pose.position.z = -fixed_frame_pos[2] + 0.04;
  temp.pose.orientation.x = 0.707;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 0.707;
  temp.scale.x = 0.085;
  temp.scale.y = 0.085;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "wheel2";
  temp.ns = "ns";
  temp.id = 23;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CYLINDER;

  cart_msg.markers.push_back(temp);


  temp.pose.position.x = 0.82;
  temp.pose.position.y = -0.285;
  temp.pose.position.z = -fixed_frame_pos[2] + 0.04;
  temp.pose.orientation.x = 0.707;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 0.707;
  temp.scale.x = 0.085;
  temp.scale.y = 0.085;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "wheel3";
  temp.ns = "ns";
  temp.id = 24;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CYLINDER;

  cart_msg.markers.push_back(temp);


  temp.pose.position.x = 0.82;
  temp.pose.position.y = 0.285;
  temp.pose.position.z = -fixed_frame_pos[2] + 0.04;
  temp.pose.orientation.x = 0.707;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 0.707;
  temp.scale.x = 0.085;
  temp.scale.y = 0.085;
  temp.scale.z = 0.03;
  temp.header.frame_id = "cart_frame";
  temp.text = "wheel4";
  temp.ns = "ns";
  temp.id = 25;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
  temp.type = visualization_msgs::Marker::CYLINDER;

  cart_msg.markers.push_back(temp);
	
}