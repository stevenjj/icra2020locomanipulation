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
  int num_clus = 12;
  int dim = 13;

  double x_max_raw = 0.4982398435516086;
  double x_min_raw = -0.1498382909533441;
  double y_max_raw = -0.2001942913049159;
  double y_min_raw = -0.3995447987545915;

  Eigen::VectorXd data_mean;
  data_mean.setZero(dim);
  Eigen::VectorXd data_std_dev;
  data_std_dev.setZero(dim);

  char filename[64];
  snprintf(filename, sizeof(char)*64, "/home/mihir/lMD/R/good/rfrh/norm.yaml");
  param_handler.load_yaml_file(filename);

  param_handler.getNestedValue({"Normalization Data", "rhx", "Mu"}, data_mean[0]);
  param_handler.getNestedValue({"Normalization Data", "rhx", "Sigma"}, data_std_dev[0]);
  param_handler.getNestedValue({"Normalization Data", "rhy", "Mu"}, data_mean[1]);
  param_handler.getNestedValue({"Normalization Data", "rhy", "Sigma"}, data_std_dev[1]);
  param_handler.getNestedValue({"Normalization Data", "rhz", "Mu"}, data_mean[2]);
  param_handler.getNestedValue({"Normalization Data", "rhz", "Sigma"}, data_std_dev[2]);
  param_handler.getNestedValue({"Normalization Data", "rhrx", "Mu"}, data_mean[3]);
  param_handler.getNestedValue({"Normalization Data", "rhrx", "Sigma"}, data_std_dev[3]);
  param_handler.getNestedValue({"Normalization Data", "rhry", "Mu"}, data_mean[4]);
  param_handler.getNestedValue({"Normalization Data", "rhry", "Sigma"}, data_std_dev[4]);
  param_handler.getNestedValue({"Normalization Data", "rhrz", "Mu"}, data_mean[5]);
  param_handler.getNestedValue({"Normalization Data", "rhrz", "Sigma"}, data_std_dev[5]);

  param_handler.getNestedValue({"Normalization Data", "rfx", "Mu"}, data_mean[6]);
  param_handler.getNestedValue({"Normalization Data", "rfx", "Sigma"}, data_std_dev[6]);
  param_handler.getNestedValue({"Normalization Data", "rfy", "Mu"}, data_mean[7]);
  param_handler.getNestedValue({"Normalization Data", "rfy", "Sigma"}, data_std_dev[7]);
  param_handler.getNestedValue({"Normalization Data", "rfz", "Mu"}, data_mean[8]);
  param_handler.getNestedValue({"Normalization Data", "rfz", "Sigma"}, data_std_dev[8]);
  param_handler.getNestedValue({"Normalization Data", "rfrz", "Mu"}, data_mean[9]);
  param_handler.getNestedValue({"Normalization Data", "rfrz", "Sigma"}, data_std_dev[9]);

  param_handler.getNestedValue({"Normalization Data", "comx", "Mu"}, data_mean[10]);
  param_handler.getNestedValue({"Normalization Data", "comx", "Sigma"}, data_std_dev[10]);
  param_handler.getNestedValue({"Normalization Data", "comy", "Mu"}, data_mean[11]);
  param_handler.getNestedValue({"Normalization Data", "comy", "Sigma"}, data_std_dev[11]);
  param_handler.getNestedValue({"Normalization Data", "pelz", "Mu"}, data_mean[12]);
  param_handler.getNestedValue({"Normalization Data", "pelz", "Sigma"}, data_std_dev[12]);

  double x_max = (x_max_raw-data_mean[6])/data_std_dev[6];
  double x_min = (x_min_raw-data_mean[6])/data_std_dev[6];
  double y_max = (y_max_raw-data_mean[7])/data_std_dev[7];
  double y_min = (y_min_raw-data_mean[7])/data_std_dev[7];

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
    list_of_mus_raw.push_back(Eigen::VectorXd::Zero(dim));
    list_of_Sigmas_raw.push_back(Eigen::MatrixXd::Zero(dim,dim));
    list_of_mus.push_back(Eigen::VectorXd::Zero(4));
    list_of_Sigmas.push_back(Eigen::MatrixXd::Zero(4,4));
  }

  for(int i = 0; i<num_clus; i++){
    param_handler.load_yaml_file(std::string("/home/mihir/lMD/R/good/rfrh/gmm") + std::to_string(i) + std::string(".yaml"));
    param_handler.getVector("Mu", mu);
    param_handler.getVector("Sigma", sigma);
    param_handler.getValue("Weight", alphs[i]);

    for(int m = 0; m < dim; m++){
      list_of_mus_raw[i][m] = mu[m];
      // std::cout << "mu = " << list_of_mus_raw[i][m] << std::endl;
    }

    for(int k = 0; k<dim; k++){
      for(int r=0; r<dim; r++){
        list_of_Sigmas_raw[i](k, r) = sigma[dim*k + r];
        // std::cout << "sigma[" << i << "]" << "[" << k << "]"<< "[" << r << "]: " << list_of_Sigmas_raw[i](k,r) << std::endl;
      }
    }
    // std::cout << "alpha[i] = " << alphs[i] << std::endl;

    for(int m = 0; m < 4; m++){
      if (m < 2) {
      list_of_mus[i][m] = mu[m];
    } else {
      list_of_mus[i][m] = mu[m+4];
    }
    }
  }

  for(int i = 0; i<num_clus; i++){
    for(int k = 0; k<4; k++){
      for(int r=0; r<4; r++){
        if (k<2 && r<2){
          list_of_Sigmas[i](k, r) = list_of_Sigmas_raw[i](k, r);
        } else if (k<2 && r>1){
          list_of_Sigmas[i](k, r) = list_of_Sigmas_raw[i](k, r+4);
        } else if (k>1 && r<2) {
          list_of_Sigmas[i](k, r) = list_of_Sigmas_raw[i](k+4, r);
        } else {
          list_of_Sigmas[i](k, r) = list_of_Sigmas_raw[i](k+4, r+4);
        }

        // std::cout << "sigma[" << i << "]" << "[" << k << "]"<< "[" << r << "]: " << list_of_Sigmas[i](k,r) << std::endl;
      }
    }
  }

  gmmfitter.setMu(list_of_mus);
  gmmfitter.setAlpha(alphs);
  gmmfitter.setSigma(list_of_Sigmas);

  Eigen::VectorXd conf = Eigen::VectorXd::Zero(4);

  for (int i=0; i<bins; i++){
    for (int k=0; k<bins; k++){
      conf[0] = 1.009210749707662;
      conf[1] = 0.3389702227712242;
      conf[2] = xspread[i];
      conf[3] = yspread[k];
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
