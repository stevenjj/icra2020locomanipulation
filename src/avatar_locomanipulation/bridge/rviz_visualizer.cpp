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