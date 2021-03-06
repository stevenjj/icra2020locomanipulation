#include <avatar_locomanipulation/helpers/param_handler.hpp>
#include <iostream>

ParamHandler::ParamHandler(){
}

ParamHandler::ParamHandler(const std::string & file_name){
  load_yaml_file(file_name);
}

void ParamHandler::load_yaml_file(const std::string & file_name){
	config = YAML::LoadFile(file_name);
}

ParamHandler::~ParamHandler(){}

bool ParamHandler::getString(const std::string & key, std::string& str_value) {
  if (config[key]){
	  str_value = config[key].as<std::string>();
	  return true;  	
	}
  return false;
}

bool ParamHandler::getVector(const std::string & key,
                             std::vector<double> & vec_value) {
  if (config[key]){
    vec_value = config[key].as<std::vector<double> >();
    return true;
  }
  return false;
}

bool ParamHandler::getValue(const std::string & key, double & double_value) {
  if (config[key]){
    double_value = config[key].as<double>();
    return true;
  }
  return false;
}

bool ParamHandler::getNestedValue(const std::vector<std::string> & nested_key, double & double_value){
	// Create a recursive implementation of this.
	YAML::Node current_node = Clone(config);
	for(int i = 0; i < nested_key.size(); i++){
		if (current_node[nested_key[i]]){
			current_node = current_node[nested_key[i]];			
		}
		else{
			//std::cerr << "Warning. The key " << nested_key[i] << " doesn't exist" << std::endl;
			return false;
		}
	}
	double_value = current_node.as<double>();
	return true;
}


bool ParamHandler::getBoolean(const std::string & key, bool & bool_value) {
  if (config[key]){
    bool_value = config[key].as<bool>();
    return true;
  }
  return false;
}

bool ParamHandler::getInteger(const std::string & key, int & int_value) {
  if (config[key]){
    int_value = config[key].as<int>();
    return true;
  }
  return false;
}

