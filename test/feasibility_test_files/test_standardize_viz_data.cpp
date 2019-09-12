#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <iostream>
#include <fstream>



int main(int argc, char ** argv){
	// Define the strings for getting and printing values
	std::string score_string;
	std::string position_string;
	// The max val to be standardized against
	double max_val, temp, standardized;
	// The x, y, z of position
	double x, y, z;
	// For the Yaml emitter	
	YAML::Emitter out;
	out << YAML::BeginMap;
	// Example loading of the file
	ParamHandler param_handler;
	param_handler.load_yaml_file("right_hand_feasibility.yaml");
	// Define the temporary position for posting to standardized yaml
	Eigen::Vector3d temp_rhand_pos;
	
	

	
	score_string = "feasibility_score_" + std::to_string(1);
	param_handler.getValue(score_string, max_val);

	// Find the Maximum feasibility score for standardization
	for(int i=2; i<=14973; ++i){
		score_string = "feasibility_score_" + std::to_string(i);
		param_handler.getValue(score_string, temp);
		if(temp >= max_val){
			max_val = temp;
		}
		std::cout << "get max\n";
	}

	// Divide all of the feasibility scores by the max
	for(int w=1; w<=14973; ++w){
		score_string = "feasibility_score_" + std::to_string(w);
		position_string = "right_hand_position_" + std::to_string(w);
		// Get, standardize, and emit the feasibility score
		param_handler.getValue(score_string, temp);
		standardized = temp/max_val;
		data_saver::emit_value(out, score_string, standardized);
		// Get and emit the rhand position
		param_handler.getNestedValue({position_string, "x"}, x);
		param_handler.getNestedValue({position_string, "y"}, y);
		param_handler.getNestedValue({position_string, "z"}, z);
		temp_rhand_pos[0] = x; temp_rhand_pos[1] = y; temp_rhand_pos[2] = z;
		data_saver::emit_position(out, position_string, temp_rhand_pos);
		std::cout << "emit\n";
	}

	out << YAML::EndMap;

	std::ofstream file_output_stream("right_hand_standardized_feasibility.yaml");
	file_output_stream << out.c_str();
}