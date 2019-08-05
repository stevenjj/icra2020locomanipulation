#include <Configuration.h>
#include <avatar_locomanipulation/helpers/gmm_fit.hpp>
#include <avatar_locomanipulation/models/valkyrie_model.hpp>
#include <avatar_locomanipulation/feasibility/feasibility.hpp>
#include <ros/ros.h>
#include <avatar_locomanipulation/bridge/val_rviz_translator.hpp>

#include <avatar_locomanipulation/helpers/orientation_utils.hpp>
#include <avatar_locomanipulation/helpers/yaml_data_saver.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>
#include <iostream>
#include <fstream>

int main(int argc, char ** argv){
  ValkyrieModel valkyrie;
  GMMFit gmmfitter;
  feasibility fiz;
  ParamHandler param_handler;

  int num_clus = 3;

  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;

  Eigen::VectorXd datum;
  datum = Eigen::VectorXd::Zero(6);

  gmmfitter.setNumClusters(num_clus);
  gmmfitter.setDim(6);
  gmmfitter.initializeNormalization();

  for(int i=0; i<1000; i++){
    char filename[64];
    // std::cout << "hello 1" << std::endl;
    snprintf(filename, sizeof(char)*32, "/home/mihir/lMD/h%d.yaml", i);
    // std::cout << "hello 2" << std::endl;
    param_handler.load_yaml_file(filename);

    // std::cout << "hello 3" << std::endl;

    param_handler.getNestedValue({"RPalmPos", "x"}, x);
    param_handler.getNestedValue({"RPalmPos", "y"}, y);
    param_handler.getNestedValue({"RPalmPos", "z"}, z);

    param_handler.getNestedValue({"RPalmOri", "rx"}, rx);
    param_handler.getNestedValue({"RPalmOri", "ry"}, ry);
    param_handler.getNestedValue({"RPalmOri", "rz"}, rz);

    datum[0] = x;
    datum[1] = y;
    datum[2] = z;
    datum[3] = rx;
    datum[4] = ry;
    datum[5] = rz;

    gmmfitter.addData(datum);
  }

  gmmfitter.prepData();

  gmmfitter.normalizeData();

  gmmfitter.randInitialGuess();
  gmmfitter.expectationMax();


  std::vector<std::string> list_of_vars;

  list_of_vars.push_back("x");
  list_of_vars.push_back("y");
  list_of_vars.push_back("z");
  list_of_vars.push_back("rx");
  list_of_vars.push_back("ry");
  list_of_vars.push_back("rz");

  YAML::Emitter out1;
  out1 << YAML::BeginMap;
    data_saver::emit_gmm_normalize(out1, gmmfitter.data_mean, gmmfitter.data_std_dev, list_of_vars);
  out1 << YAML::EndMap;
  char filename1[64];
  snprintf(filename1, sizeof(char)*32, "/home/mihir/lMD/norm.yaml");
  std::ofstream file_output_stream1(filename1);
  file_output_stream1 << out1.c_str();

  std::vector<Eigen::VectorXd> list_of_mus;
  std::vector<Eigen::MatrixXd> list_of_Sigmas;
  Eigen::VectorXd alphas;

  Eigen::VectorXd data_min;
  Eigen::VectorXd data_max;

  list_of_mus = gmmfitter.list_of_mus;
  list_of_Sigmas = gmmfitter.list_of_Sigmas;
  data_min = gmmfitter.data_min;
  data_max = gmmfitter.data_max;
  alphas = gmmfitter.alphs;

  for(int i=0; i<num_clus; i++){
  YAML::Emitter out;
  out<< YAML::BeginMap;
    data_saver::emit_joint_configuration(out,"Mu",list_of_mus[i]);
    data_saver::emit_gmm_sigma(out,"Sigma",list_of_Sigmas[i]);
    out << YAML::Key << "Weight" << YAML::Value << alphas[i];
  out << YAML::EndMap;
  char filename[64];
  // filename = "file.yaml"
  snprintf(filename, sizeof(char)*32, "/home/mihir/lMD/gmmR%d.yaml", i);
  std::ofstream file_output_stream(filename);
  file_output_stream << out.c_str();
  }
  YAML::Emitter out;
  out<< YAML::BeginMap;
    data_saver::emit_joint_configuration(out,"Max",data_max);
    data_saver::emit_joint_configuration(out,"Min",data_min);
  out << YAML::EndMap;
  std::ofstream file_output_stream("/home/mihir/lMD/dataminmaxR.yaml");
  file_output_stream << out.c_str();
}
