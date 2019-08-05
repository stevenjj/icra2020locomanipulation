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

int main(int argc , char ** argv){
  ValkyrieModel valkyrie;
  GMMFit gmmfitter;
  feasibility fiz;
  ParamHandler param_handler;

  int bins = 20;
  int num_clus = 3;

  double x_max_raw = 0.6231378694367828;
  double x_min_raw = -0.5992914253933654;
  double y_max_raw = 0.0147501494355215;
  double y_min_raw = -0.8688573531158277;

  // double x_max_raw = 1.0;
  // double x_min_raw = -1.0;
  // double y_max_raw = 0.5;
  // double y_min_raw = -1.0;

  gmmfitter.setDim(6);
  gmmfitter.initializeNormalization();

  Eigen::VectorXd datum;
  datum = Eigen::VectorXd::Zero(6);
  std::vector<Eigen::VectorXd> list_of_datums;

  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;


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


  gmmfitter.prepData();
  gmmfitter.normalizeData();

  list_of_datums = gmmfitter.list_of_datums;

  double x_max = (x_max_raw-gmmfitter.data_mean[0])/gmmfitter.data_std_dev[0];
  double x_min = (x_min_raw-gmmfitter.data_mean[0])/gmmfitter.data_std_dev[0];
  double y_max = (y_max_raw-gmmfitter.data_mean[1])/gmmfitter.data_std_dev[1];
  double y_min = (y_min_raw-gmmfitter.data_mean[1])/gmmfitter.data_std_dev[1];


  gmmfitter.setNumClusters(num_clus);
  gmmfitter.setDim(2);

  Eigen::VectorXd xspread = Eigen::VectorXd::LinSpaced(bins, x_min, x_max);
  Eigen::VectorXd yspread = Eigen::VectorXd::LinSpaced(bins, y_min, y_max);
  Eigen::MatrixXd prob_grid = Eigen::MatrixXd::Zero(bins,bins);

  std::vector<Eigen::VectorXd> list_of_mus;
  std::vector<Eigen::MatrixXd> list_of_Sigmas;
  std::vector<Eigen::VectorXd> list_of_mus_raw;
  std::vector<Eigen::MatrixXd> list_of_Sigmas_raw;
  Eigen::VectorXd alphs = Eigen::VectorXd::Zero(num_clus);
  std::vector<double>  mu;
  std::vector<double>  sigma;


  for(int i = 0; i < num_clus; i++){
    list_of_mus_raw.push_back(Eigen::VectorXd::Zero(6));
    list_of_Sigmas_raw.push_back(Eigen::MatrixXd::Zero(6,6));
    list_of_mus.push_back(Eigen::VectorXd::Zero(2));
    list_of_Sigmas.push_back(Eigen::MatrixXd::Zero(2,2));
  }

  for(int i = 0; i<num_clus; i++){
    param_handler.load_yaml_file(std::string("/home/mihir/lMD/gmmR") + std::to_string(i) + std::string(".yaml"));
    param_handler.getVector("Mu", mu);
    param_handler.getVector("Sigma", sigma);
    param_handler.getValue("Weight", alphs[i]);

    for(int m = 0; m < 6; m++){
      list_of_mus_raw[i][m] = mu[m];
      // std::cout << "mu = " << list_of_mus[i][m] << std::endl;
    }

    for(int k = 0; k<6; k++){
      for(int r=0; r<6; r++){
        list_of_Sigmas_raw[i](k, r) = sigma[6*k + r];
        // std::cout << "sigma[" << i << "]" << "[" << k << "]"<< "[" << r << "]: " << list_of_Sigmas_raw[i](k,r) << std::endl;
      }
    }
    // std::cout << "alpha[i] = " << alphs[i] << std::endl;

    for(int m = 0; m < 2; m++){
      list_of_mus[i][m] = mu[m];
    }
  }

  for(int i = 0; i<num_clus; i++){
    for(int k = 0; k<2; k++){
      for(int r=0; r<2; r++){
        list_of_Sigmas[i](k, r) = list_of_Sigmas_raw[i](k, r);
        // std::cout << "sigma[" << i << "]" << "[" << k << "]"<< "[" << r << "]: " << list_of_Sigmas[i](k,r) << std::endl;
      }
    }
  }

  gmmfitter.setMu(list_of_mus);
  gmmfitter.setAlpha(alphs);
  gmmfitter.setSigma(list_of_Sigmas);

  Eigen::VectorXd conf = Eigen::VectorXd::Zero(2);

  for (int i=0; i<bins; i++){
    for (int k=0; k<bins; k++){
      conf[0] = xspread[i];
      conf[1] = yspread[k];
      prob_grid(i,k) = gmmfitter.mixtureModelProb(conf);
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
