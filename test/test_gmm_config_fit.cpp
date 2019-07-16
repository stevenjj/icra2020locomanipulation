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

  int task;

  Eigen::Vector3d lhand_cur_pos;
  Eigen::Quaternion<double> lhand_cur_ori;
  Eigen::Vector3d lhand_cur_ori_vec;

  Eigen::VectorXd datum;
  datum = Eigen::VectorXd::Zero(6);

  std::vector<Eigen::VectorXd> list_of_datums;

  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;

  int num_clus = 5;

  task = 2;
if (task == 1){
  for(int i=0; i<1000; i++){
    fiz.generateRandomRightArm();
    lhand_cur_pos=fiz.lhand_cur_pos;
    lhand_cur_ori=fiz.lhand_cur_ori;

    // std::cout << "x:" << rhand_cur_ori.x()
    //           << "y:" << rhand_cur_ori.y()
    //           << "z:" << rhand_cur_ori.z()
    //           << "w:" << rhand_cur_ori.w() << std::endl;
    // rhand_cur_ori_aa = rhand_cur_ori;
    // rhand_cur_ori_vec = rhand_cur_ori_aa.axis()*rhand_cur_ori_aa.angle();

    // std::cout << "before: " << std::endl;
    // std::cout << "  angle " << rhand_cur_ori_aa.angle() << std::endl;
    // std::cout << "  axis " << rhand_cur_ori_aa.axis().transpose() << std::endl;

    math_utils::convert(lhand_cur_ori, lhand_cur_ori_vec);

    // std::cout << "after: " << std::endl;
    // std::cout << "  angle " << rhand_cur_ori_vec.norm() << std::endl;
    // std::cout << "  axis " << (rhand_cur_ori_vec / rhand_cur_ori_vec.norm()).transpose() << std::endl;

  //  std::string filename = "THIS_PACKAGE_PATH../../some_folder"

    YAML::Emitter out;
    out<< YAML::BeginMap;
      data_saver::emit_position(out,"LPalmPos",lhand_cur_pos);
      data_saver::emit_orientation_vector(out,"LPalmOri",lhand_cur_ori_vec);
    out << YAML::EndMap;
    char filename[64];
    // filename = "file.yaml"
    snprintf(filename, sizeof(char)*32, "/home/mihir/lMD/h%d.yaml", i);
    std::ofstream file_output_stream(filename);
    file_output_stream << out.c_str();
  }
} else if (task == 2){

  datum = Eigen::VectorXd::Zero(6);
  gmmfitter.setNumClusters(num_clus);
  gmmfitter.setDim(6);
  gmmfitter.initializeNormalization();
  for(int i=0; i<1000; i++){
    char filename[64];
    snprintf(filename, sizeof(char)*32, "/home/mihir/lMD/l%d.yaml", i);
    param_handler.load_yaml_file(filename);

    param_handler.getNestedValue({"LPalmPos", "x"}, x);
    param_handler.getNestedValue({"LPalmPos", "y"}, y);
    param_handler.getNestedValue({"LPalmPos", "z"}, z);

    param_handler.getNestedValue({"LPalmOri", "rx"}, rx);
    param_handler.getNestedValue({"LPalmOri", "ry"}, ry);
    param_handler.getNestedValue({"LPalmOri", "rz"}, rz);

    datum[0] = x;
    datum[1] = y;
    datum[2] = z;
    datum[3] = rx;
    datum[4] = ry;
    datum[5] = rz;

    gmmfitter.addData(datum);
  }

  gmmfitter.prepData();


  // gmmfitter.useRawData();
  // gmmfitter.randInitialGuess();
  // gmmfitter.expectationMax();

  gmmfitter.normalizeData();
  gmmfitter.randInitialGuess();
  gmmfitter.expectationMax();

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
  snprintf(filename, sizeof(char)*32, "/home/mihir/lMD/gmmL%d.yaml", i);
  std::ofstream file_output_stream(filename);
  file_output_stream << out.c_str();
}
YAML::Emitter out;
out<< YAML::BeginMap;
  data_saver::emit_joint_configuration(out,"Max",data_max);
  data_saver::emit_joint_configuration(out,"Min",data_min);
out << YAML::EndMap;
std::ofstream file_output_stream("/home/mihir/lMD/dataminmax.yaml");
file_output_stream << out.c_str();
} else if (task==3){
  int bins = 20;
  int samples_per_bin = 1000;

  double x_max = 0.6231378694367828;
  double x_min = -0.5992914253933654;
  double y_max = 0.0147501494355215;
  double y_min = -0.8688573531158277;

  gmmfitter.setNumClusters(num_clus);
  gmmfitter.setDim(6);
  gmmfitter.initializeNormalization();

  Eigen::VectorXd xspread = Eigen::VectorXd::LinSpaced(bins, x_min, x_max);
  Eigen::VectorXd yspread = Eigen::VectorXd::LinSpaced(bins, y_min, y_max);
  Eigen::VectorXd handcon = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd rand_hand = Eigen::VectorXd::Zero(6);

  Eigen::MatrixXd prob_grid = Eigen::MatrixXd::Zero(bins,bins);

  std::cout << "hello? 1" << std::endl;

  // load some data
  for(int i=0; i<1000; i++){
    char filename[64];
    snprintf(filename, sizeof(char)*32, "/home/mihir/lMD/h%d.yaml", i);
    param_handler.load_yaml_file(filename);

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

    // list_of_datums.push_back(datum);

    gmmfitter.addData(datum);
  }

  std::cout << "hello? 2" << std::endl;


  gmmfitter.prepData();
  gmmfitter.normalizeData();

  std::cout << "hello? 3" << std::endl;


  list_of_datums = gmmfitter.list_of_datums;

  std::vector<Eigen::VectorXd> list_of_mus;
  std::vector<Eigen::MatrixXd> list_of_Sigmas;
  Eigen::VectorXd alphs = Eigen::VectorXd::Zero(num_clus);
  std::vector<double>  mu;
  std::vector<double>  sigma;

  for(int i = 0; i < num_clus; i++){
    list_of_mus.push_back(Eigen::VectorXd::Zero(6));
    list_of_Sigmas.push_back(Eigen::MatrixXd::Zero(6,6));
  }

  for(int i = 0; i<num_clus; i++){
    param_handler.load_yaml_file(std::string("/home/mihir/lMD/gmm") + std::to_string(i) + std::string(".yaml"));
    param_handler.getVector("Mu", mu);
    param_handler.getVector("Sigma", sigma);
    param_handler.getValue("Weight", alphs[i]);

    for(int m = 0; m < mu.size(); m++){
      list_of_mus[i][m] = mu[m];
      // std::cout << "mu = " << list_of_mus[i][m] << std::endl;
    }

    for(int k = 0; k<6; k++){
      for(int r=0; r<6; r++){
        list_of_Sigmas[i](k, r) = sigma[6*k + r];
        // std::cout << "sigma[i][k][r] = " << list_of_Sigmas[i](k,r) << std::endl;
      }
    }
  }



  gmmfitter.setMu(list_of_mus);
  gmmfitter.setAlpha(alphs);
  gmmfitter.setSigma(list_of_Sigmas);

  double cell_p;

  Eigen::VectorXd z_norm = Eigen::VectorXd::Zero(6);

  for (int i=0; i<bins; i++){
    for (int k=0; k<bins; k++){
      cell_p = 0.0;
      for (int r=0; r<samples_per_bin; r++){
        rand_hand = list_of_datums[rand() % 1000];
        handcon[0] = xspread[i];
        handcon[1] = yspread[k];
        handcon[2] = rand_hand[2];
        handcon[3] = rand_hand[3];
        handcon[4] = rand_hand[4];
        handcon[5] = rand_hand[5];

        z_norm = (handcon - gmmfitter.data_mean).cwiseQuotient(gmmfitter.data_std_dev);

        handcon[0] = z_norm[0];
        handcon[1] = z_norm[1];

        cell_p+=gmmfitter.mixtureModelProb(handcon);

      }
      prob_grid(i,k) = cell_p/((double)samples_per_bin);
      std::cout << "bin(i,k) = (" << i << ", " << k << ") done" << std::endl;
    }
  }

  std::cout << "\n";
  for(int i = 0; i < bins; i++){
    for(int j = 0; j < bins; j++){
      std::cout << prob_grid(i, j) << "," ;
    }
    std::cout << "\n";
  }

}
return 0;

}
