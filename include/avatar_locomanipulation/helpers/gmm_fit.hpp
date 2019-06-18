#ifndef GMM_FIT_H
#define GMM_FIT_H

#include <Eigen/Dense>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>

class GMMFit{
public:
  int dim;
  int num_clus;
  int num_data;
  double pi = 3.159265358979323846;
  Eigen::MatrixXd data;

  std::vector<Eigen::VectorXd> list_of_datums;

  GMMFit();
  GMMFit(const Eigen::MatrixXd & data_in);

  ~GMMFit();

  void setData(const std::vector<Eigen::VectorXd> & data_in);
  double multivariateGuassian(Eigen::VectorXd & x, Eigen::VectorXd & mu, Eigen::MatrixXd & Sigma);
  // void expectationMax();
};

#endif
