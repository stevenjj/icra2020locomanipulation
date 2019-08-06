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

  int num_clus = 1;

  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;

  Eigen::VectorXd datum;
  datum = Eigen::VectorXd::Zero(1);

  gmmfitter.setNumClusters(num_clus);
  gmmfitter.setDim(1);
  gmmfitter.initializeNormalization();

  std::string filename;

  std::cout << "about to read" << std::endl;
  for(int i=0; i<1000; i++){
    filename =  std::string(THIS_PACKAGE_PATH) + "../lMD/h" + std::to_string(i) + ".yaml";
    std::cout << filename << std::endl;
    param_handler.load_yaml_file(filename);

    std::cout << "reading file i = "  << i << std::endl;
    // std::cout << "hello 3" << std::endl;

    param_handler.getNestedValue({"LPalmPos", "x"}, x);
    param_handler.getNestedValue({"LPalmPos", "y"}, y);
    // param_handler.getNestedValue({"LPalmPos", "z"}, z);

    // param_handler.getNestedValue({"LPalmOri", "rx"}, rx);
    // param_handler.getNestedValue({"LPalmOri", "ry"}, ry);
    // param_handler.getNestedValue({"LPalmOri", "rz"}, rz);

    datum[0] = x;
    // datum[1] = y;
    // datum[2] = z;
    // datum[3] = rx;
    // datum[4] = ry;
    // datum[5] = rz;

    gmmfitter.addData(datum);
  }

  // Perform a GMM Fit
  gmmfitter.prepData();
  gmmfitter.normalizeData();
  gmmfitter.randInitialGuess();
  gmmfitter.expectationMax();

  // Post GMM Fit:
  // test logCost for all the data:
  std::cout << "Testing the log cost for all the data" << std::endl;
  std::cout << "  scaling factor sum = " << gmmfitter.getScalingFactorSum() << std::endl;
  Eigen::VectorXd datum_normalized;
  double datum_log_cost = 0.0;
  for(int i = 0; i < 1000; i++){
    filename =  std::string(THIS_PACKAGE_PATH) + "../lMD/h" + std::to_string(i) + ".yaml";
    // std::cout << filename << std::endl;
    param_handler.load_yaml_file(filename);

    // std::cout << "reading file i = "  << i << std::endl;

    param_handler.getNestedValue({"LPalmPos", "x"}, x);
    // param_handler.getNestedValue({"LPalmPos", "y"}, y);
    // param_handler.getNestedValue({"LPalmPos", "z"}, z);

    // param_handler.getNestedValue({"LPalmOri", "rx"}, rx);
    // param_handler.getNestedValue({"LPalmOri", "ry"}, ry);
    // param_handler.getNestedValue({"LPalmOri", "rz"}, rz);

    datum[0] = x;
    // datum[1] = y;
    // datum[2] = z;
    // datum[3] = rx;
    // datum[4] = ry;
    // datum[5] = rz;

    gmmfitter.normalizeInputCalculate(datum, datum_normalized);

    datum_log_cost = gmmfitter.logCost(datum_normalized); 
    std::cout << "i:" << i << " log cost = " << datum_log_cost << " Is non-negative: " << (datum_log_cost >= 0 ? "True" : "False") << std::endl;
  }


  std::vector<std::string> list_of_vars;

  list_of_vars.push_back("x");
  // list_of_vars.push_back("y");
  // list_of_vars.push_back("z");
  // list_of_vars.push_back("rx");
  // list_of_vars.push_back("ry");
  // list_of_vars.push_back("rz");

  YAML::Emitter out1;
  out1 << YAML::BeginMap;
    data_saver::emit_gmm_normalize(out1, gmmfitter.data_mean, gmmfitter.data_std_dev, list_of_vars);
  out1 << YAML::EndMap;
  filename =  std::string(THIS_PACKAGE_PATH) + "../lMD/norm.yaml";

  std::ofstream file_output_stream1(filename);
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

  filename =  std::string(THIS_PACKAGE_PATH) + "../lMD/gmmL" + std::to_string(i) + ".yaml";
  std::ofstream file_output_stream(filename);
  file_output_stream << out.c_str();
  }
  YAML::Emitter out;
  out<< YAML::BeginMap;
    data_saver::emit_joint_configuration(out,"Max",data_max);
    data_saver::emit_joint_configuration(out,"Min",data_min);
  out << YAML::EndMap;
  std::ofstream file_output_stream(THIS_PACKAGE_PATH"../lMD/dataminmaxL.yaml");
  file_output_stream << out.c_str();
}
