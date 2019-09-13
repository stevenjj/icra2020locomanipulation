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
	interpolated_pose_pub = n->advertise<geometry_msgs::PoseArray>(interpolated_pose_pub_topic, 10);
	s_value_pub = n->advertise<std_msgs::Float64>(s_value_pub_topic, 10);

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
void RVizVisualizer::visualizeConfigurationTrajectory(const Eigen::VectorXd & q_start_in, TrajEuclidean & traj_q_in, bool visualize_once){
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

 		if (visualize_once){
  			break; // break if we want to visualize only once
  		}
		// Visualize the robot on a loop otherwise

  	}  	


}

void RVizVisualizer::visualizeConfigurationTrajectory(const Eigen::VectorXd & q_start_in, TrajEuclidean & traj_q_in, std::vector<double> & s_traj,  bool visualize_once){
	std::cout << "[RVizVisualizer] Visualizing configuration trajectory with s variable" << std::endl;

	// Initialize Starting Configuration
	setStartConfig(q_start_in);
	populateStartConfigJointMsg();

	// Set loop rate for the actual time of the trajectory
	ros::Rate loop_rate(1.0/traj_q_in.get_dt());

  	// Publish Forever until terminated
  	while (ros::ok()){
  		for(int i = 0; i < traj_q_in.get_trajectory_length(); i++){
	  		// get the current position
	  		traj_q_in.get_pos(i, q_current);

	  		// get the current s value
	  		s_value_msg.data = s_traj[i];	  		

	  		// Set configuration
  			setCurrentConfig(q_current);
		    populateCurrentConfigJointMsg();

		    // Do visualization
			br_robot->sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
			robot_joint_state_pub.publish(joint_msg_init);

			br_ik->sendTransform(tf::StampedTransform(tf_world_pelvis_current, ros::Time::now(), "world", "val_ik_robot/pelvis"));
			robot_ik_joint_state_pub.publish(joint_msg_current);

	  		s_value_pub.publish(s_value_msg);

	  		// Spin and SLeep
		    ros::spinOnce();
		    loop_rate.sleep();  			
  		}

 		if (visualize_once){
  			break; // break if we want to visualize only once
  		}
		// Visualize the robot on a loop otherwise

  	}  	
}

void RVizVisualizer::visualizeConfigurationTrajectory(std::shared_ptr<ManipulationFunction> f_s, int robot_manipulation_side, 
													  const Eigen::VectorXd & q_start_in, TrajEuclidean & traj_q_in, bool visualize_once){

	std::cout << "[RVizVisualizer] Visualizing configuration trajectory" << std::endl;

	// Initialize Starting Configuration
	setStartConfig(q_start_in);
	populateStartConfigJointMsg();

	// Set loop rate for the actual time of the trajectory
	// ros::Rate loop_rate(pub_freq);
	// ros::Rate loop_rate(5.0);
	ros::Rate loop_rate(1.0/traj_q_in.get_dt());


	// Prepare hand pose trajectory visualization message
	geometry_msgs::PoseArray hand_poses;
	hand_poses.header.frame_id = "world";
	geometry_msgs::Pose pose;

	Eigen::Vector3d hand_pos;
	Eigen::Quaterniond hand_ori;
	int N_viz_hand_pose = 100;

	double s = 0;
	for(int i = 0; i < N_viz_hand_pose; i++){
		s = ((double) (i))/((double) N_viz_hand_pose);
		f_s->getPose(s, hand_pos, hand_ori);		

		pose.position.x = hand_pos[0];
		pose.position.y = hand_pos[1];
		pose.position.z = hand_pos[2];
		pose.orientation.x = hand_ori.x();
		pose.orientation.y = hand_ori.y();
		pose.orientation.z = hand_ori.z();
		pose.orientation.w = hand_ori.w();
		hand_poses.poses.push_back(pose);
	}


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

			interpolated_pose_pub.publish(hand_poses);

	  		// Spin and SLeep
		    ros::spinOnce();
		    loop_rate.sleep();  			
  		}

 		if (visualize_once){
  			break; // break if we want to visualize only once
  		}
		// Visualize the robot on a loop otherwise
  	}  	

}