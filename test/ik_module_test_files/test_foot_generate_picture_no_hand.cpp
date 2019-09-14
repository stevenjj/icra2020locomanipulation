#include <avatar_locomanipulation/walking/config_trajectory_generator.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>
#include "pinocchio/utils/timer.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

// YAML
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>


// Stance Generation
#include <avatar_locomanipulation/ik_module/valkyrie_stance_generation.hpp>
#include <avatar_locomanipulation/data_types/manipulation_function.hpp>

// Standard
#include <iostream>
#include <math.h>
#include <cassert>


void initialize_config(Eigen::VectorXd & q_init, std::shared_ptr<RobotModel> & valkyrie){
  // X dimensional state vectors
  Eigen::VectorXd q_start;

  // std::cout << "valkyrie->getDimQ(): " << valkyrie->getDimQ() << std::endl;
  // Set origin at 0.0
  q_start = Eigen::VectorXd::Zero(valkyrie->getDimQ());

  double theta = 0.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

  Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0); //Initialized to remember the w component comes first
  init_quat = aa;

  q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w(); // Set up the quaternion in q
  q_start[2] = 1.0; // set z value to 1.0, this is the pelvis location

  q_start[valkyrie->getJointIndex("leftHipPitch")] = -0.3;
  q_start[valkyrie->getJointIndex("rightHipPitch")] = -0.3;
  q_start[valkyrie->getJointIndex("leftKneePitch")] = 0.6;
  q_start[valkyrie->getJointIndex("rightKneePitch")] = 0.6;
  q_start[valkyrie->getJointIndex("leftAnklePitch")] = -0.3;
  q_start[valkyrie->getJointIndex("rightAnklePitch")] = -0.3;

  q_start[valkyrie->getJointIndex("rightShoulderPitch")] = 0.07;
  q_start[valkyrie->getJointIndex("rightShoulderRoll")] = 1.19;
  q_start[valkyrie->getJointIndex("rightElbowPitch")] = 1.6 ; //0.4;
  q_start[valkyrie->getJointIndex("rightForearmYaw")] = -0.37;

  q_start[valkyrie->getJointIndex("leftShoulderPitch")] = -0.2;
  q_start[valkyrie->getJointIndex("leftShoulderRoll")] = -1.1;
  q_start[valkyrie->getJointIndex("leftElbowPitch")] = -0.4;
  q_start[valkyrie->getJointIndex("leftForearmYaw")] = 1.5;

  q_init = q_start;
}


struct PositionBool{
	Eigen::Vector3d pos;
	bool b;
};




void visualizeResults(std::vector<PositionBool> & pb, visualization_msgs::MarkerArray & msg){
	visualization_msgs::Marker temp;

	std::cout << "pb.size(): " << pb.size() << std::endl;

	double factor = 0.95;
	for(int i=0; i<pb.size(); ++i){

		// Fill the Marker 
		temp.pose.position.x = pb[i].pos[0];
		temp.pose.position.y = pb[i].pos[1];
		temp.pose.position.z = pb[i].pos[2];	
		temp.pose.orientation.x = 0.;	
		temp.pose.orientation.y = 0.;
		temp.pose.orientation.z = 0.;
		temp.pose.orientation.w = 1.;
		temp.ns = "foot_positions";
		temp.id = i;
		temp.scale.x = 0.05*factor;
		temp.scale.y = 0.05*factor;
		temp.scale.z = 0.02*factor;
		temp.header.frame_id = "world";
		if(pb[i].b == true){
			temp.color.r = 0.0f;
			temp.color.b = 0.0f;
			temp.color.g = 1.0f;
		}else{
			temp.color.r = 1.0f;
			temp.color.b = 0.0f;
			temp.color.g = 0.0f;
		}
		temp.color.a = 1.0;
		temp.action = visualization_msgs::Marker::ADD;
		temp.type = visualization_msgs::Marker::CUBE;
		msg.markers.push_back(temp);
	}

	


}





void test_hand_in_place_config_trajectory_generator(){
  std::cout << "[Running Config Trajectory Generator Test] In place walking with hand constant" << std::endl;

  std::string urdf_filename = THIS_PACKAGE_PATH"models/valkyrie_no_fingers.urdf";
  std::shared_ptr<RobotModel> valkyrie_model(new RobotModel(urdf_filename));

  int N_resolution = 60; 
  ConfigTrajectoryGenerator ctg(valkyrie_model, N_resolution);

  Eigen::VectorXd q_start, q_end;
  initialize_config(q_start, valkyrie_model);

  // Test initial configuration computation for flat ground
  ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_1);
  ctg.computeInitialConfigForFlatGround(q_start, q_end);

  // Create visualization object
  std::shared_ptr<ros::NodeHandle> ros_node(std::make_shared<ros::NodeHandle>());
  RVizVisualizer visualizer(ros_node, valkyrie_model);  

  // Update Initial
  q_start = q_end;
  valkyrie_model->updateFullKinematics(q_start);

  // In place step test with keeping the hand in place
  // Create footsteps in place
  Footstep footstep_1; footstep_1.setLeftSide();
  Footstep footstep_2; footstep_2.setRightSide();

  valkyrie_model->getFrameWorldPose("leftCOP_Frame", footstep_1.position, footstep_1.orientation);  
  valkyrie_model->getFrameWorldPose("rightCOP_Frame", footstep_2.position, footstep_2.orientation);  

  Eigen::Vector3d original_pos;
  original_pos = footstep_1.position;

  double theta = M_PI/6.0;
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Quaternion<double> quat_angleaxis(aa); //Initialized to remember the w component comes first


	double x_min = -0.8;
	double x_max = 0.8;
	double y_min = -0.1;
	double y_max = 0.4;
	double dx = 0.05;
	// double x_min = 0.0;
	// double x_max = 0.1;
	// double y_min = 0.0;
	// double y_max = 0.1;
	// double dx = 0.05;
	int N = static_cast<int>((x_max - x_min) / dx);
	int M = static_cast<int>((y_max - y_min) / dx);
	double x_, y_;

	PositionBool temp;
	std::vector<PositionBool> pb;
	ros::Rate rate(1000);
	ros::NodeHandle n;

	ros::Publisher foot_markers_pub = n.advertise<visualization_msgs::MarkerArray>("foot_positions", 100);

	visualization_msgs::MarkerArray msg;


  for(int i=0; i<N+1; ++i){ 
  	// Get x pos at this step
  	x_ = x_min + i*dx;

  	for(int j=0; j <M+1; ++j){
  		// Reset the lfoot position 
  		footstep_1.position = original_pos;
  		// Add this current x_ to the foot pos
  		footstep_1.position[0] += x_;
  		// Get y pos at this step
  		y_ = y_min + j*dx;
  		// Add this current y_ to the foot pos
  		footstep_1.position[1] += y_;

  		footstep_1.setPosOri(footstep_1.position, quat_angleaxis);
		  footstep_2.setPosOri(footstep_2.position, quat_angleaxis);

		  // std::vector<Footstep> input_footstep_list = {footstep_2, footstep_1};
		  std::vector<Footstep> input_footstep_list = {footstep_1};

		  ctg.setUseRightHand(false);
		  ctg.setUseTorsoJointPosition(false);
		  ctg.reinitializeTaskStack();

		    // timer
		  PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);


		  // Have fast double support times
		  ctg.wpg.setDoubleSupportTime(0.2);
		  // Set Verbosity
		  ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2);
		  // Solve for configurations
		  timer.tic();
		  temp.b = ctg.computeConfigurationTrajectory(q_start, input_footstep_list);
		  temp.pos = footstep_1.position;
		  pb.push_back(temp);
		  std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

		  visualizeResults(pb, msg);
  		foot_markers_pub.publish(msg);
			rate.sleep();
			ros::spinOnce();

			// Visualize Trajectory
  		visualizer.visualizeConfigurationTrajectory(q_start, ctg.traj_q_config, true);
  		
  	}
  }  
  // NOW RERUN THIS ONCE TO GET IT INTO A GOOD POSITION

  // Reset the lfoot position 
  		footstep_1.position = original_pos;
  		// Add this current x_ to the foot pos
  		footstep_1.position[0] -= 0.4;
  		// Get y pos at this step
  		// Add this current y_ to the foot pos
  		footstep_1.position[1] += 0.1;

  		footstep_1.setPosOri(footstep_1.position, quat_angleaxis);
		  footstep_2.setPosOri(footstep_2.position, quat_angleaxis);

		  // std::vector<Footstep> input_footstep_list = {footstep_2, footstep_1};
		  std::vector<Footstep> input_footstep_list = {footstep_1};


		  // Get current hand pose
		  Eigen::Vector3d rhand_pos;
		  Eigen::Quaterniond rhand_ori;
		  valkyrie_model->getFrameWorldPose("rightPalm", rhand_pos, rhand_ori);  
		  // Set Constant Right Hand trajectory to current //To do. set task gain for hand to be small in orientation
		  ctg.setConstantRightHandTrajectory(rhand_pos, rhand_ori);
		  ctg.setUseRightHand(true);
		  ctg.setUseTorsoJointPosition(false);
		  ctg.reinitializeTaskStack();

		    // timer
		  PinocchioTicToc timer = PinocchioTicToc(PinocchioTicToc::MS);


		  // Have fast double support times
		  ctg.wpg.setDoubleSupportTime(0.2);
		  // Set Verbosity
		  ctg.setVerbosityLevel(CONFIG_TRAJECTORY_VERBOSITY_LEVEL_2);
		  // Solve for configurations
		  timer.tic();
		  temp.b = ctg.computeConfigurationTrajectory(q_start, input_footstep_list);
		  temp.pos = footstep_1.position;
		  pb.push_back(temp);
		  std::cout << "IK Trajectory took: " << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

		  visualizer.visualizeConfigurationTrajectory(q_start, ctg.traj_q_config, true);

////////////////////////// AFTER THIS WAS ADDDED FOR ROSBAG

	ros::Rate loop_rate(20);
	Eigen::VectorXd q_finish;
	ctg.traj_q_config.get_pos(ctg.getDiscretizationSize() - 1, q_finish);

	// Joint State Publisher
  ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
  ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);
  // Initialize Rviz translator
  ValRvizTranslator rviz_translator;

  // Transform broadcaster
  tf::TransformBroadcaster      br_ik;
  tf::TransformBroadcaster      br_robot;

  // Initialize Transforms and Messages
  tf::Transform tf_world_pelvis_init;
  tf::Transform tf_world_pelvis_end;

  sensor_msgs::JointState joint_msg_init;
  sensor_msgs::JointState joint_msg_end;

  // Visualize q_start and q_end in RVIZ
  rviz_translator.populate_joint_state_msg(valkyrie_model->model, q_start, tf_world_pelvis_init, joint_msg_init);
  rviz_translator.populate_joint_state_msg(valkyrie_model->model, q_finish, tf_world_pelvis_end, joint_msg_end);

	while(ros::ok()){
		foot_markers_pub.publish(msg);

		br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
		robot_joint_state_pub.publish(joint_msg_init);

		br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
		robot_ik_joint_state_pub.publish(joint_msg_end);
		ros::spinOnce();
		loop_rate.sleep();

	}

}



int main(int argc, char ** argv){   
  ros::init(argc, argv, "test_footstep_probability");
  test_hand_in_place_config_trajectory_generator();

  return 0;
}