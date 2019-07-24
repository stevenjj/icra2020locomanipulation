#ifndef ALM_RVIZ_VISUALIZER_H
#define ALM_RVIZ_VISUALIZER_H

#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/rviz_translator.hpp>
#include <avatar_locomanipulation/models/robot_model.hpp>

class RVizVisualizer{
public:
	RVizVisualizer();
	RVizVisualizer(std::shared_ptr<ros::NodeHandle> & n_input);
	~RVizVisualizer();

    void setRobotModel(std::shared_ptr<RobotModel> & robot_model_input);
	void setNodeHandle(std::shared_ptr<ros::NodeHandle> & n_input);

    // Robot Model object
    std::shared_ptr<RobotModel> robot_model;

    // The translator object
 	RvizTranslator rviz_translator;

 	// ROS 
    std::shared_ptr<ros::NodeHandle> n;
    ros::Publisher robot_ik_joint_state_pub;
    ros::Publisher robot_joint_state_pub;

    std::shared_ptr<tf::TransformBroadcaster> br_ik;
    std::shared_ptr<tf::TransformBroadcaster> br_robot;

    // Variable Containers
	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_end;
	sensor_msgs::JointState joint_msg_init;
	sensor_msgs::JointState joint_msg_end;

	std::string robot_ik_joint_pub_topic = "robot1/joint_states";
	std::string robot_joint_pub_topic = "robot2/joint_states";

    Eigen::VectorXd q_start;
    Eigen::VectorXd q_current;    


};





#endif 