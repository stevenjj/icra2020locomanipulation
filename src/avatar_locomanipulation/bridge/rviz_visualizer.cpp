#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>

RVizVisualizer::RVizVisualizer(){	
	std::cout << "[RVizVisualizer] constructed" << std::endl;
}

RVizVisualizer::RVizVisualizer(std::shared_ptr<ros::NodeHandle> & n_input){
	n = n_input;
}


RVizVisualizer::~RVizVisualizer(){	
	std::cout << "[RVizVisualizer] destroyed" << std::endl;
}

void RVizVisualizer::setRobotModel(std::shared_ptr<RobotModel> & robot_model_input){
	// Assign the robot model
	robot_model = robot_model_input;
	// Initialize the values of the joints to 0
	q_start = Eigen::VectorXd::Zero(robot_model->getDimQ());
	q_current = Eigen::VectorXd::Zero(robot_model->getDimQ());
}


void RVizVisualizer::setNodeHandle(std::shared_ptr<ros::NodeHandle> & n_input){
	// Set the node handle
	n = n_input;
	// Advertise topic after we have a node handle
	robot_ik_joint_state_pub = n->advertise<sensor_msgs::JointState>(robot_ik_joint_pub_topic, 10);
	robot_joint_state_pub = n->advertise<sensor_msgs::JointState>(robot_joint_pub_topic, 10);

	// Create transform broadcaster objects after we have a node handle
	br_ik = std::make_shared<tf::TransformBroadcaster>();
	br_robot = std::make_shared<tf::TransformBroadcaster>();
}

