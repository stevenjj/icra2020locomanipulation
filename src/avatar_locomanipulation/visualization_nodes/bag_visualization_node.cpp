#include <avatar_locomanipulation/visualization_nodes/bag_visualization_node.hpp>



BagVisualizationNode::BagVisualizationNode(ros::NodeHandle* n_input, const std::string & parameter_file_input){
	s_current = 0;
	nh = n_input;
	parameter_file = parameter_file_input;
	std::cout << "BagVisualizationNode Created\n";
	getParameters();
}


BagVisualizationNode::~BagVisualizationNode(){

}

void BagVisualizationNode::s_callback(const std_msgs::Float64ConstPtr & msg){

	s_current = msg->data;
}


void BagVisualizationNode::getParameters(){
	ParamHandler param_handler;

	param_handler.load_yaml_file(THIS_PACKAGE_PATH"hand_trajectory/bag_lift_putdown_parameters.yaml");

	param_handler.getValue("radius", radius);

	param_handler.getValue("rotation_angle", rotation_angle);

	param_handler.getNestedValue({"center_fixed_frame_position", "x"}, center_frame_pos[0]);
	param_handler.getNestedValue({"center_fixed_frame_position", "y"}, center_frame_pos[1]);
	param_handler.getNestedValue({"center_fixed_frame_position", "z"}, center_frame_pos[2]);

	param_handler.getValue("pose_spacing", pose_spacing);

	param_handler.getValue("lift_height", lift_height);

	double rx, ry, rz, rw;
	param_handler.getNestedValue({"hand_initial_orientation", "x"}, rx);
	param_handler.getNestedValue({"hand_initial_orientation", "y"}, ry);
	param_handler.getNestedValue({"hand_initial_orientation", "z"}, rz);
	param_handler.getNestedValue({"hand_initial_orientation", "w"}, rw);

	q_init.x() = rx; q_init.y() = ry; q_init.z() = rz; q_init.w() = rw;
	q_init.normalize();

	param_handler.getNestedValue({"center_fixed_frame_orientation", "x"}, rx);
	param_handler.getNestedValue({"center_fixed_frame_orientation", "y"}, ry);
	param_handler.getNestedValue({"center_fixed_frame_orientation", "z"}, rz);
	param_handler.getNestedValue({"center_fixed_frame_orientation", "w"}, rw);

	q_fixed.x() = rx; q_fixed.y() = ry; q_fixed.z() = rz; q_fixed.w() = rw;
	q_fixed.normalize();
}


void BagVisualizationNode::fillBagMarkerArray(visualization_msgs::MarkerArray & bag_msg){

	visualization_msgs::Marker temp;

  // First generate the top of handle of the bag
  temp.pose.position.x = 0.0;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = 0.0;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.01;
  temp.scale.y = 0.08;
  temp.scale.z = 0.01;
  temp.header.frame_id = "bag_frame";
  temp.text = "handle_top";
  temp.ns = "ns";
  temp.id = 1;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
	temp.type = visualization_msgs::Marker::CUBE;

	bag_msg.markers.push_back(temp);

	// Next generate the side handle of the bag
  temp.pose.position.x = 0.0;
  temp.pose.position.y = 0.04;
  temp.pose.position.z = -0.04;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.01;
  temp.scale.y = 0.01;
  temp.scale.z = 0.08;
  temp.header.frame_id = "bag_frame";
  temp.text = "handle_side_1";
  temp.ns = "ns";
  temp.id = 2;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
	temp.type = visualization_msgs::Marker::CUBE;

	bag_msg.markers.push_back(temp);

	// Next generate the other side handle of the bag
  temp.pose.position.x = 0.0;
  temp.pose.position.y = -0.04;
  temp.pose.position.z = -0.04;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.01;
  temp.scale.y = 0.01;
  temp.scale.z = 0.08;
  temp.header.frame_id = "bag_frame";
  temp.text = "handle_side_2";
  temp.ns = "ns";
  temp.id = 3;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 1.0;
  temp.action = visualization_msgs::Marker::ADD;
	temp.type = visualization_msgs::Marker::CUBE;

	bag_msg.markers.push_back(temp);


	// Last generate the main part of the bag
  temp.pose.position.x = 0.0;
  temp.pose.position.y = 0.0;
  temp.pose.position.z = -0.18;
  temp.pose.orientation.x = 0.0;
  temp.pose.orientation.y = 0.0;
  temp.pose.orientation.z = 0.0;
  temp.pose.orientation.w = 1.0;
  temp.scale.x = 0.08;
  temp.scale.y = 0.16;
  temp.scale.z = 0.20;
  temp.header.frame_id = "bag_frame";
  temp.text = "main_bag";
	temp.ns = "ns";
	temp.id = 4;
  temp.color.r = 0.5f;
  temp.color.g = 0.5f;
  temp.color.b = 0.5f;
  temp.color.a = 0.8;
  temp.action = visualization_msgs::Marker::ADD;
	temp.type = visualization_msgs::Marker::CUBE;

	bag_msg.markers.push_back(temp);
}


void BagVisualizationNode::getVizInformation(tf::Transform & tf_world_center, tf::Transform & tf_center_bag){

	// Define the fixed frame transform
	tf_world_center.setOrigin(tf::Vector3(center_frame_pos[0], center_frame_pos[1], center_frame_pos[2]));
	tf_world_center.setRotation(tf::Quaternion(q_fixed.x(), q_fixed.y(), q_fixed.z(), q_fixed.w()));

	// First need to determine the bounds of s for when we are lifting vs when we are turning
	double arc_length = rotation_angle * radius;

	N = static_cast<int>(arc_length / pose_spacing);// Number of wps to describe the arc
  M = static_cast<int>(lift_height / pose_spacing); // Number of wps to describe each of the up and down motions

  int total_wps = N + (2*M);

  // set the frame from world to fixed frame
  tf_world_center.setOrigin(tf::Vector3(center_frame_pos[0], center_frame_pos[1], center_frame_pos[2]));
	tf_world_center.setRotation(tf::Quaternion(q_fixed.x(), q_fixed.y(), q_fixed.z(), q_fixed.w()));

	Eigen::Quaternion<double> q, q_i;	

  double percent_lifting = (static_cast<double>(M) / static_cast<double>(total_wps));
  double percent_turning = (static_cast<double>(N) / static_cast<double>(total_wps));
  // Now we know the first and last bit of s inZ [0,1] that will constitute lifting/putting down vs turning

  if(s_current < percent_lifting){ // then the motion of the frame is going to be up
  	
  	// x and y constant wrt the fixed frame
  	// z is the start height + s/s_lift*lift_height
  			// Thus when s = s_lift, we are 1*lift_height
  	tf_center_bag.setOrigin(tf::Vector3(radius, 0.0,((s_current / percent_lifting) * lift_height) ));
  	// In this portion of the movement, there is no change in angle of the bag
  	tf_center_bag.setRotation(tf::Quaternion(0., 0., 0., 1.));

  }
  else if(s_current > (1. - percent_lifting)){ // the the motion of the frame is going to be down
  	// x and y constant wrt the fixed frame
  	// z is the final height -                       //s/s_lift*lift_height
  			// Thus when s = s_lift, we are 1*lift_height
  	tf_center_bag.setOrigin(tf::Vector3(radius * cos(rotation_angle), 0.0, lift_height - (((s_current - (1 - percent_lifting)) / percent_lifting) * lift_height) ));
  	// In this portion of the movement, there is no change in angle of the bag
  	tf_center_bag.setRotation(tf::Quaternion(0., 0., 0., 1.));
  }
  else{ // the motion is on the arc
  	// Need to define an s_local s.t. s = percent_lifting -> s_local = 0
  	// and s = 1 - percent_lifting -> s_local = 1
  	double s_local = (s_current - percent_lifting)/((1. - percent_lifting) - percent_lifting);

  	q.x() = 0.0; q.y() = 0.0; q.z() = sin((s_local * rotation_angle) / 2); q.w() = cos((s_local * rotation_angle) / 2);

  	q_i = q * q_fixed;

  	// x and y are changing while z is constant
  	tf_center_bag.setOrigin(tf::Vector3((radius * cos( (s_local * rotation_angle) ) ), (radius * sin( (s_local * rotation_angle) ) ), lift_height));
  	// rotation occurs with the waypoints
  	tf_center_bag.setRotation(tf::Quaternion(q_i.x(), q_i.y(), q_i.z(), q_i.w()));
  } 

}	