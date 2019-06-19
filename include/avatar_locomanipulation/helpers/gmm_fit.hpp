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
  double pi = 3.14159265358979323846;

  std::vector<Eigen::VectorXd> list_of_datums;
  std::vector<Eigen::VectorXd> list_of_mus;
  std::vector<Eigen::MatrixXd> list_of_Sigmas;
  Eigen::VectorXd alphs;
  Eigen::MatrixXd gam;
  Eigen::VectorXd n;

  Eigen::VectorXd mu_sum;
  Eigen::MatrixXd sig_sum;

  GMMFit();
  GMMFit(const std::vector<Eigen::VectorXd> & data_in, const int & num_clus_in);

  ~GMMFit();



  void setData(const std::vector<Eigen::VectorXd> & data_in);
  double multivariateGuassian(Eigen::VectorXd & x, Eigen::VectorXd & mu, Eigen::MatrixXd & Sigma);
  void expectStep();
  void maxStep();
  double logLike();
  void expectationMax();
  void setDim(const int & dim_in);
  void setNumClusters(const int & num_clus_in);
  void setMu(const std::vector<Eigen::VectorXd> & list_of_mus_in);
  void setSigma(const std::vector<Eigen::MatrixXd> & list_of_Sigmas_in);
  void setAlpha(const Eigen::VectorXd & alphs_in);
  void randInitialGuess();
};

#endif
