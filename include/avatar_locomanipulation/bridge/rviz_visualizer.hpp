#ifndef ALM_RVIZ_VISUALIZER_H
#define ALM_RVIZ_VISUALIZER_H

#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/rviz_translator.hpp>
#include <avatar_locomanipulation/models/robot_model.hpp>
#include <avatar_locomanipulation/data_types/trajectory_SE3.hpp>
#include <avatar_locomanipulation/data_types/manipulation_function.hpp>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

class RVizVisualizer{
public:
	RVizVisualizer();
    RVizVisualizer(std::shared_ptr<ros::NodeHandle> & n_input, std::shared_ptr<RobotModel> & robot_model_input);
	RVizVisualizer(std::shared_ptr<ros::NodeHandle> & n_input);
	~RVizVisualizer();

    void setRobotModel(std::shared_ptr<RobotModel> & robot_model_input);
	void setNodeHandle(std::shared_ptr<ros::NodeHandle> & n_input);

    void setStartConfig(const Eigen::VectorXd & q_start_in);
    void setCurrentConfig(const Eigen::VectorXd & q_current_in);
    void setPubFreq(const double & pub_freq_in);

    void visualizeConfiguration(const Eigen::VectorXd & q_start_in, const Eigen::VectorXd & q_current_in);
    void visualizeConfigurationTrajectory(const Eigen::VectorXd & q_start_in, TrajEuclidean & traj_q_in, bool visualize_once=false);
    void visualizeConfigurationTrajectory(std::shared_ptr<ManipulationFunction> f_s, int robot_manipulation_side, const Eigen::VectorXd & q_start_in, TrajEuclidean & traj_q_in, bool visualize_once=false);

    void populateStartConfigJointMsg();
    void populateCurrentConfigJointMsg();

    // Robot Model object
    std::shared_ptr<RobotModel> robot_model;

    // The translator object
 	RvizTranslator rviz_translator;

 	// ROS 
    std::shared_ptr<ros::NodeHandle> n;
    ros::Publisher robot_ik_joint_state_pub;
    ros::Publisher robot_joint_state_pub;
    ros::Publisher interpolated_pose_pub;

    std::shared_ptr<tf::TransformBroadcaster> br_ik;
    std::shared_ptr<tf::TransformBroadcaster> br_robot;

    double pub_freq = 20.0; // Hz

    // Variable Containers
	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_current;

	sensor_msgs::JointState joint_msg_init;
	sensor_msgs::JointState joint_msg_current;

	std::string robot_ik_joint_pub_topic = "robot1/joint_states";
	std::string robot_joint_pub_topic = "robot2/joint_states";
    std::string interpolated_pose_pub_topic = "interp_output";


    Eigen::VectorXd q_start;
    Eigen::VectorXd q_current;    


};





#endif 