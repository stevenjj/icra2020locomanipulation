#include <iostream>
#include <avatar_locomanipulation/cubic_interpolation_module/six_dim_vec.hpp>
#include <vector>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"


int main(int argc, char ** argv){

	// Initialize ROS node for publishing joint messages
	ros::init(argc, argv, "test_rviz_interpolator");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);

	// Initialize Rviz translator
	ValRvizTranslator rviz_translator;

	tf::TransformBroadcaster 	  br_hinge;
	tf::TransformBroadcaster      br_ik;
	tf::TransformBroadcaster      br_robot;

	// Joint State Publisher
	ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
	ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);
	// Door Publisher
	ros::Publisher door_pub = n.advertise<visualization_msgs::Marker>("door", 100);
	// Handle Publisher
	ros::Publisher handle_pub = n.advertise<visualization_msgs::Marker>("handle", 100);
	// Hinge Publisher
	ros::Publisher hinge_pub = n.advertise<visualization_msgs::Marker>("hinge", 100);
	// Interpolation Output Publisher
	ros::Publisher interpolated_pose_pub = n.advertise<geometry_msgs::PoseArray>("interp_output", 100);

	// Initialize Transforms and Messages
	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_end;
	tf::Transform tf_world_hinge;

	sensor_msgs::JointState joint_msg_init; 
	sensor_msgs::JointState joint_msg_end; 
	visualization_msgs::Marker door_msg, handle_msg, hinge_msg, temp;
	geometry_msgs::PoseArray hand_poses;
	geometry_msgs::Pose pose;

	std::cout << "Initialize Valkyrie Model" << std::endl;
	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
	std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
	std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";

 	RobotModel valkyrie(filename, meshDir, srdf_filename);
	Eigen::VectorXd  q_start(valkyrie.getDimQ()); q_start.setZero();
	Eigen::VectorXd  q_end(valkyrie.getDimQ()); q_end.setZero();

	// Initialize Linear positions
	q_start[2] = 1.13177;
	q_end[0] = -1.0;
	q_end[2] = q_start[2];	
	// Initialize Quaternion components to Identity
	q_start[6] = 1.0;
	q_end[6] = 1.0;

	// Set q_end:
	q_end[valkyrie.getJointIndex("leftHipPitch")] = -0.3; 
	q_end[valkyrie.getJointIndex("rightHipPitch")] = -0.3;  
	q_end[valkyrie.getJointIndex("leftKneePitch")] = 0.6;  
	q_end[valkyrie.getJointIndex("rightKneePitch")] = 0.6;
	q_end[valkyrie.getJointIndex("leftAnklePitch")] = -0.3; 
	q_end[valkyrie.getJointIndex("rightAnklePitch")] = -0.3;

	q_end[valkyrie.getJointIndex("rightShoulderPitch")] = -0.2; 
	q_end[valkyrie.getJointIndex("rightShoulderRoll")] = 1.1;  
	q_end[valkyrie.getJointIndex("rightElbowPitch")] = 0.4;  
	q_end[valkyrie.getJointIndex("rightForearmYaw")] = 1.5; 	
	q_end[valkyrie.getJointIndex("leftShoulderPitch")] = -0.2; 
	q_end[valkyrie.getJointIndex("leftShoulderRoll")] = -1.1;  
	q_end[valkyrie.getJointIndex("leftElbowPitch")] = -0.4;
	q_end[valkyrie.getJointIndex("leftForearmYaw")] = 1.5;  
 

	// Visualize q_start and q_end in RVIZ
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_start, tf_world_pelvis_init, joint_msg_init);
	rviz_translator.populate_joint_state_msg(valkyrie.model, q_end, tf_world_pelvis_end, joint_msg_end);

	// Add the Hinge frame
	tf_world_hinge.setOrigin(tf::Vector3(1.5, -0.5, 1.125));
	tf_world_hinge.setRotation(tf::Quaternion(0, 0, 0.707, 0.707));


	// Fill the Handle marker msg
	handle_msg.pose.position.x = 1.475;
	handle_msg.pose.position.y = 0.4;
	handle_msg.pose.position.z = 1.125;
	handle_msg.pose.orientation.y = 0.;
	handle_msg.pose.orientation.z = 0.;
	handle_msg.pose.orientation.w = 1.;
	handle_msg.scale.x = 0.05;
	handle_msg.scale.y = 0.1;
	handle_msg.scale.z = 0.03;
	handle_msg.header.frame_id = "world";
	handle_msg.text = "box";
	// handle_msg.header.stamp = ros::Time::now();
	handle_msg.color.r = 1.0f;
	handle_msg.color.g = 0.0f;
	handle_msg.color.b = 0.0f;
	handle_msg.color.a = 1.0;
	// handle_msg.lifetime = ros::Duration();
	handle_msg.action = visualization_msgs::Marker::ADD;
	handle_msg.type = visualization_msgs::Marker::CUBE;

	// Fill the Hinge marker msg
	hinge_msg.pose.position.x = 1.5;
	hinge_msg.pose.position.y = -0.5;
	hinge_msg.pose.position.z = 1.125;
	hinge_msg.pose.orientation.y = 0.;
	hinge_msg.pose.orientation.z = 0.;
	hinge_msg.pose.orientation.w = 1.;
	hinge_msg.scale.x = 0.05;
	hinge_msg.scale.y = 0.03;
	hinge_msg.scale.z = 0.1;
	hinge_msg.header.frame_id = "world";
	hinge_msg.text = "box";
	// hinge_msg.header.stamp = ros::Time::now();
	hinge_msg.color.r = 1.0f;
	hinge_msg.color.g = 0.0f;
	hinge_msg.color.b = 0.0f;
	hinge_msg.color.a = 1.0;
	// hinge_msg.lifetime = ros::Duration();
	hinge_msg.action = visualization_msgs::Marker::ADD;
	hinge_msg.type = visualization_msgs::Marker::CUBE;

	// Fill the Door marker msg
	door_msg.pose.position.x = 1.5;
	door_msg.pose.position.y = 0.;
	door_msg.pose.position.z = 1.125;
	door_msg.pose.orientation.y = 0.;
	door_msg.pose.orientation.z = 0.;
	door_msg.pose.orientation.w = 1.;
	door_msg.scale.x = 0.05;
	door_msg.scale.y = 1.0;
	door_msg.scale.z = 2.25;
	door_msg.header.frame_id = "world";
	door_msg.text = "box";
	// door_msg.header.stamp = ros::Time::now();
	door_msg.color.r = 0.0f;
	door_msg.color.g = 1.0f;
	door_msg.color.b = 0.0f;
	door_msg.color.a = 1.0;
	// door_msg.lifetime = ros::Duration();
	door_msg.action = visualization_msgs::Marker::ADD;
	door_msg.type = visualization_msgs::Marker::CUBE;


	std::string file = THIS_PACKAGE_PATH"hand_trajectory/door_trajectory.yaml";

	std::vector<std::vector<double> > outs;

	std::shared_ptr<SixDimVec> example; 
	example = std::shared_ptr<SixDimVec>(new SixDimVec(file));


	hand_poses.header.frame_id = "hinge_frame";
	double s;
	double radius = 0.9;

	for(int i=0; i<100; ++i){
		s = ((double) (i))/100.0;

		std::cout << "s = " << s << std::endl;


		example->evaluate(s);

		// Fill the PoseArray hand_poses
		pose.position.x = (example->pos_out[0])*radius;
		pose.position.y = (example->pos_out[1])*radius;
		pose.position.z = example->pos_out[2];
		pose.orientation.x = example->quat_out.x();
		pose.orientation.y = example->quat_out.y();
		pose.orientation.z = example->quat_out.z();
		pose.orientation.w = example->quat_out.w();
		hand_poses.poses.push_back(pose);
		
	}

	std::cout << "hand_poses.poses.size(): " << hand_poses.poses.size() << std::endl;

	for(int k=0; k<hand_poses.poses.size(); ++k){
		// std::cout << "hand_poses.poses[k].position.x: " << hand_poses.poses[k].position.x << std::endl;
		// std::cout << "hand_poses.poses[k].position.y: " << hand_poses.poses[k].position.y << std::endl;
		// std::cout << "hand_poses.poses[k].position.z: " << hand_poses.poses[k].position.z << std::endl;
		// std::cout << "hand_poses.poses[k].orientation.x: " << hand_poses.poses[k].orientation.x << std::endl;
		// std::cout << "hand_poses.poses[k].orientation.y: " << hand_poses.poses[k].orientation.y << std::endl;
		// std::cout << "hand_poses.poses[k].orientation.z: " << hand_poses.poses[k].orientation.z << std::endl;
		// std::cout << "hand_poses.poses[k].orientation.w: " << hand_poses.poses[k].orientation.w << std::endl;
	}


	while (ros::ok()){
		br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
	    robot_joint_state_pub.publish(joint_msg_init);

	    br_hinge.sendTransform(tf::StampedTransform(tf_world_hinge, ros::Time::now(), "world", "hinge_frame"));
	    
	    door_pub.publish(door_msg);
	    handle_pub.publish(handle_msg);
	    hinge_pub.publish(hinge_msg);
	    interpolated_pose_pub.publish(hand_poses);

	    br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
	    robot_ik_joint_state_pub.publish(joint_msg_end);

		ros::spinOnce();
		loop_rate.sleep();		
	}
	return 0;
}




// void conversion(std::vector<std::vector<double> > & outputs){

// 	for(int i=0; i<outputs.size(); ++i){

// 		Eigen::Vector3d axisangle;
// 		axisangle[0] = outputs[i][3];
// 		axisangle[1] = outputs[i][4];
// 		axisangle[2] = outputs[i][5];

// 	}
// }