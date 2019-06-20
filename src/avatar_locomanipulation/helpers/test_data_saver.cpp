// Package Path Definition
#include <Configuration.h>

// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>
#include <avatar_locomanipulation/models/valkyrie_model.hpp>

#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

#include <avatar_locomanipulation/feasibility/feasibility.hpp>

#include <iostream>
#include <fstream>

int main(int argc, char **argv){
  ValkyrieModel valkyrie;
  Eigen::VectorXd  q_data(valkyrie.getDimQ()); q_data.setZero();
  Eigen::VectorXd  q_start(valkyrie.getDimQ()); q_data.setZero();

  feasibility	fiz;

  for (int ii = 0; ii<10; ii++){
  fiz.InverseKinematicsTop(300);
  q_data = fiz.q_data;
  q_start = fiz.q_start;

  YAML::Emitter out;
  out<< YAML::BeginMap;
    data_saver::emit_joint_configuration(out,"InitialConfiguration",q_start);
    data_saver::emit_joint_configuration(out,"FinalConfiguration",q_data);
  out << YAML::EndMap;
  char filename[64];
  // filename = "file.yaml"
  snprintf(filename, sizeof(char)*32, "file%d.yaml", ii);
  std::ofstream file_output_stream(filename);
  file_output_stream << out.c_str();
  return 0;
  }
}

//rosrun avatar_locomanipulation test_data_saver
