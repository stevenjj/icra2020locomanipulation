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

	void setNodeHandle(std::shared_ptr<ros::NodeHandle> & n_input);

    std::shared_ptr<ros::NodeHandle> n;
 	RvizTranslator rviz_translator;

	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_end;

	sensor_msgs::JointState joint_msg_init;
	sensor_msgs::JointState joint_msg_end;

};





#endif 