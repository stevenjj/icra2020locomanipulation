#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>

RVizVisualizer::RVizVisualizer(){	
	std::cout << "[RVizVisualizer] constructed" << std::endl;
}

// Need to initialize the robot_model shared pointer pointer as part of the list initialization
RVizVisualizer::RVizVisualizer(std::shared_ptr<ros::NodeHandle> & n_input, std::shared_ptr<RobotModel> & robot_model_input):robot_model(robot_model_input){
	setRobotModel(robot_model);
	setNodeHandle(n_input);
}


RVizVisualizer::RVizVisualizer(std::shared_ptr<ros::NodeHandle> & n_input){
	setNodeHandle(n_input);
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

void RVizVisualizer::setStartConfig(const Eigen::VectorXd & q_start_in){
	q_start = q_start_in;	
}

void RVizVisualizer::setCurrentConfig(const Eigen::VectorXd & q_current_in){	
	q_current = q_current_in;
}

void RVizVisualizer::populateStartConfigJointMsg(){	
	rviz_translator.populate_joint_state_msg( robot_model->model, q_start, tf_world_pelvis_init, joint_msg_init);
}

void RVizVisualizer::populateCurrentConfigJointMsg(){	
	rviz_translator.populate_joint_state_msg(robot_model->model, q_current, tf_world_pelvis_current, joint_msg_current);
}

void RVizVisualizer::setPubFreq(const double & pub_freq_in){
	pub_freq = pub_freq_in;
}

void RVizVisualizer::visualizeConfiguration(const Eigen::VectorXd & q_start_in, const Eigen::VectorXd & q_current_in){
	std::cout << "[RVizVisualizer] Visualizing starting and ending configurations" << std::endl;
	// Set the configurations
	setStartConfig(q_start_in);
	setCurrentConfig(q_current_in);

	// Populate Messages
	populateStartConfigJointMsg();
    populateCurrentConfigJointMsg();

    // Set Loop Rate
	ros::Rate loop_rate(pub_freq);
  	
  	// Publish Forever until terminated
  	while (ros::ok()){
		br_robot->sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
		robot_joint_state_pub.publish(joint_msg_init);

		br_ik->sendTransform(tf::StampedTransform(tf_world_pelvis_current, ros::Time::now(), "world", "val_ik_robot/pelvis"));
		robot_ik_joint_state_pub.publish(joint_msg_current);

	    ros::spinOnce();
	    loop_rate.sleep();
  	}


}

// Visualize Configuration Trajectory
void RVizVisualizer::visualizeConfigurationTrajectory(const Eigen::VectorXd & q_start_in, TrajEuclidean & traj_q_in){
	std::cout << "[RVizVisualizer] Visualizing configuration trajectory" << std::endl;

	// Initialize Starting Configuration
	setStartConfig(q_start_in);
	populateStartConfigJointMsg();

	// Set loop rate for the actual time of the trajectory
	// ros::Rate loop_rate(pub_freq);
	// ros::Rate loop_rate(5.0);
	ros::Rate loop_rate(1.0/traj_q_in.get_dt());

  	// Publish Forever until terminated
  	while (ros::ok()){
  		for(int i = 0; i < traj_q_in.get_trajectory_length(); i++){
	  		// get the current position
	  		traj_q_in.get_pos(i, q_current);

	  		// Set configuration
  			setCurrentConfig(q_current);
		    populateCurrentConfigJointMsg();

		    // Do visualization
			br_robot->sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
			robot_joint_state_pub.publish(joint_msg_init);

			br_ik->sendTransform(tf::StampedTransform(tf_world_pelvis_current, ros::Time::now(), "world", "val_ik_robot/pelvis"));
			robot_ik_joint_state_pub.publish(joint_msg_current);

	  		// Spin and SLeep
		    ros::spinOnce();
		    loop_rate.sleep();  			
  		}

  	}  	

	// Visualize the robot on a loop

}