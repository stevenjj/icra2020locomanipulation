#include <avatar_locomanipulation/feasibility/feasibility_data_playback.hpp>
#include <avatar_locomanipulation/bridge/rviz_visualizer.hpp>

int main(int argc, char ** argv){
	// Load robot model
	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified.urdf"; 
  	std::shared_ptr<RobotModel> robot_model(new RobotModel(filename));

  	// Create playback object
	FeasibilityDataPlayBack feas_data_playback;
	feas_data_playback.setRobotModel(robot_model);

	std::string param_filepath = THIS_PACKAGE_PATH"data_generation_yaml_configurations/right_hand_right_stance.yaml";
	std::string data_filepath = THIS_PACKAGE_PATH"test/feasibility_test_files/right_hand_right_foot_s95_1.yaml";

	feas_data_playback.loadParamFile(param_filepath);
	feas_data_playback.loadData(data_filepath);	

	return 0;
}