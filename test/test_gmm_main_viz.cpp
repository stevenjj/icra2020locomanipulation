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
  int dim = 13;

  double x_max_raw = 0.9814862262380606;
  double x_min_raw = -1.091100777982219;
  double y_max_raw = 1.299264131319151;
  double y_min_raw = -0.3775383957685604;

  Eigen::VectorXd data_mean;
  data_mean.setZero(dim);
  Eigen::VectorXd data_std_dev;
  data_std_dev.setZero(dim);

  gmmfitter.setDim(dim);
  gmmfitter.initializeNormalization();

  char filename[64];
  snprintf(filename, sizeof(char)*64, "/home/mihir/lMD/1/good/lflh/norm.yaml");
  param_handler.load_yaml_file(filename);

  param_handler.getNestedValue({"Normalization Data", "lhx", "Mu"}, data_mean[0]);
  param_handler.getNestedValue({"Normalization Data", "lhx", "Sigma"}, data_std_dev[0]);
  param_handler.getNestedValue({"Normalization Data", "lhy", "Mu"}, data_mean[1]);
  param_handler.getNestedValue({"Normalization Data", "lhy", "Sigma"}, data_std_dev[1]);
  param_handler.getNestedValue({"Normalization Data", "lhz", "Mu"}, data_mean[2]);
  param_handler.getNestedValue({"Normalization Data", "lhz", "Sigma"}, data_std_dev[2]);
  param_handler.getNestedValue({"Normalization Data", "lhrx", "Mu"}, data_mean[3]);
  param_handler.getNestedValue({"Normalization Data", "lhrx", "Sigma"}, data_std_dev[3]);
  param_handler.getNestedValue({"Normalization Data", "lhry", "Mu"}, data_mean[4]);
  param_handler.getNestedValue({"Normalization Data", "lhry", "Sigma"}, data_std_dev[4]);
  param_handler.getNestedValue({"Normalization Data", "lhrz", "Mu"}, data_mean[5]);
  param_handler.getNestedValue({"Normalization Data", "lhrz", "Sigma"}, data_std_dev[5]);

  param_handler.getNestedValue({"Normalization Data", "lfx", "Mu"}, data_mean[6]);
  param_handler.getNestedValue({"Normalization Data", "lfx", "Sigma"}, data_std_dev[6]);
  param_handler.getNestedValue({"Normalization Data", "lfy", "Mu"}, data_mean[7]);
  param_handler.getNestedValue({"Normalization Data", "lfy", "Sigma"}, data_std_dev[7]);
  param_handler.getNestedValue({"Normalization Data", "lfz", "Mu"}, data_mean[8]);
  param_handler.getNestedValue({"Normalization Data", "lfz", "Sigma"}, data_std_dev[8]);
  param_handler.getNestedValue({"Normalization Data", "lfrz", "Mu"}, data_mean[9]);
  param_handler.getNestedValue({"Normalization Data", "lfrz", "Sigma"}, data_std_dev[9]);

  param_handler.getNestedValue({"Normalization Data", "comx", "Mu"}, data_mean[10]);
  param_handler.getNestedValue({"Normalization Data", "comx", "Sigma"}, data_std_dev[10]);
  param_handler.getNestedValue({"Normalization Data", "comy", "Mu"}, data_mean[11]);
  param_handler.getNestedValue({"Normalization Data", "comy", "Sigma"}, data_std_dev[11]);
  param_handler.getNestedValue({"Normalization Data", "pelz", "Mu"}, data_mean[12]);
  param_handler.getNestedValue({"Normalization Data", "pelz", "Sigma"}, data_std_dev[12]);

}
