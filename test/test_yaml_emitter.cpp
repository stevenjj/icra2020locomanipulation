#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <iostream>
#include <fstream>

int main(int argc, char ** argv){
  Eigen::Vector3d pelvis_pos(0,0,1);
  Eigen::Quaterniond pelvis_ori(0.707,0,0,0.707);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(10); 

  YAML::Emitter out;
  out << YAML::BeginMap;
    data_saver::emit_position(out, "pelvis_position", pelvis_pos);
    data_saver::emit_orientation(out, "pelvis_orientation", pelvis_ori);
    data_saver::emit_joint_configuration(out, q);
  out << YAML::EndMap;

  std::cout << "Here's the output YAML:\n" << out.c_str() << "\n" << std::endl;
  std::ofstream file_output_stream("sample_emission.yaml");
  file_output_stream << out.c_str();

  // Example loading of the file
  ParamHandler param_handler;
  param_handler.load_yaml_file("sample_emission.yaml");

  // Example of retreiving a nested value
  double stored_val;
  stored_val = param_handler.getNestedValue({"pelvis_position", "z"}, stored_val);
  std::cout << "pelvis_position z = " << stored_val << std::endl;

  return 0;
}