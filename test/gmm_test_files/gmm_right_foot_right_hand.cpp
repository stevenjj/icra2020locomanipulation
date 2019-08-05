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
  // std::string filename = THIS_PACKAGE_PATH"models/valkyrie_simplified_collisions.urdf";
  // std::string srdf_filename = THIS_PACKAGE_PATH"models/valkyrie_disable_collisions.srdf";
  // std::string meshDir  = THIS_PACKAGE_PATH"../val_model/";
  //
  // RobotModel valkyrie(filename, meshDir, srdf_filename);
  GMMFit gmmfitter;
  // feasibility fiz;
  ParamHandler param_handler;

  int prob_dim = 13;
  int num_clus = 12;

  double fx;
  double fy;
  double fz;
  double frz;

  double hx;
  double hy;
  double hz;
  double hrx;
  double hry;
  double hrz;

  double comx;
  double comy;
  double pelz;

  Eigen::VectorXd datum;
  datum = Eigen::VectorXd::Zero(prob_dim);

  gmmfitter.setNumClusters(num_clus);
  gmmfitter.setDim(prob_dim);
  gmmfitter.initializeNormalization();

  for(int i=0; i<1000; i++){
    char filename[64];
    snprintf(filename, sizeof(char)*64, "/home/mihir/lMD/R/good/l%d.yaml", i);
    param_handler.load_yaml_file(filename);

    param_handler.getNestedValue({"RPalmPos", "x"}, hx);
    param_handler.getNestedValue({"RPalmPos", "y"}, hy);
    param_handler.getNestedValue({"RPalmPos", "z"}, hz);

    param_handler.getNestedValue({"RPalmOri", "rx"}, hrx);
    param_handler.getNestedValue({"RPalmOri", "ry"}, hry);
    param_handler.getNestedValue({"RPalmOri", "rz"}, hrz);

    param_handler.getNestedValue({"RFootPos", "x"}, fx);
    param_handler.getNestedValue({"RFootPos", "y"}, fy);
    param_handler.getNestedValue({"RFootPos", "z"}, fz);
    param_handler.getNestedValue({"RFootOri", "rz"}, frz);

    param_handler.getNestedValue({"CoM", "x"}, comx);
    param_handler.getNestedValue({"CoM", "y"}, comy);
    param_handler.getNestedValue({"PelvisPos", "z"}, pelz);
    datum[0] = hx;
    datum[1] = hy;
    datum[2] = hz;
    datum[3] = hrx;
    datum[4] = hry;
    datum[5] = hrz;
    datum[6] = fx;
    datum[7] = fy;
    datum[8] = fz;
    datum[9] = frz;
    datum[10] = comx;
    datum[11] = comy;
    datum[12] = pelz;


    gmmfitter.addData(datum);
    std::cout << i << std::endl;
  }

  gmmfitter.prepData();
  gmmfitter.normalizeData();

  gmmfitter.randInitialGuess();
  gmmfitter.expectationMax();

  std::vector<std::string> list_of_vars;

  list_of_vars.push_back("rhx");
  list_of_vars.push_back("rhy");
  list_of_vars.push_back("rhz");
  list_of_vars.push_back("rhrx");
  list_of_vars.push_back("rhry");
  list_of_vars.push_back("rhrz");
  list_of_vars.push_back("rfx");
  list_of_vars.push_back("rfy");
  list_of_vars.push_back("rfz");
  list_of_vars.push_back("rfrz");
  list_of_vars.push_back("comx");
  list_of_vars.push_back("comy");
  list_of_vars.push_back("pelz");

  YAML::Emitter out1;
  out1 << YAML::BeginMap;
    data_saver::emit_gmm_normalize(out1, gmmfitter.data_mean, gmmfitter.data_std_dev, list_of_vars);
  out1 << YAML::EndMap;
  char filename1[64];
  snprintf(filename1, sizeof(char)*64, "/home/mihir/lMD/R/good/rfrh/norm.yaml");
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
  snprintf(filename, sizeof(char)*64, "/home/mihir/lMD/R/good/rfrh/gmm%d.yaml", i);
  std::ofstream file_output_stream(filename);
  file_output_stream << out.c_str();
  }
  YAML::Emitter out;
  out<< YAML::BeginMap;
    data_saver::emit_joint_configuration(out,"Max",data_max);
    data_saver::emit_joint_configuration(out,"Min",data_min);
  out << YAML::EndMap;
  std::ofstream file_output_stream("/home/mihir/lMD/R/good/rfrh/dataminmax.yaml");
  file_output_stream << out.c_str();
}
