#ifndef GMM_FIT_H
#define GMM_FIT_H

#include <Eigen/Dense>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <avatar_locomanipulation/helpers/pseudo_inverse.hpp>


class GMMFit{
public:
  int dim;
  int num_clus;
  double num_data;
  double pi = M_PI; //3.14159265358979323846; // M_PI;

  std::vector<Eigen::VectorXd> list_of_datums_raw;
  std::vector<Eigen::VectorXd> list_of_datums;
  std::vector<Eigen::VectorXd> list_of_mus;
  std::vector<Eigen::MatrixXd> list_of_Sigmas;

  std::vector<Eigen::MatrixXd> list_of_Sigma_inverses;
  std::vector<double> list_of_Sigma_determinants;


  Eigen::VectorXd alphs;
  Eigen::MatrixXd gam;
  Eigen::VectorXd n;

  Eigen::VectorXd data_mean;
  Eigen::VectorXd data_mean_sum;
  Eigen::VectorXd data_min;
  Eigen::VectorXd data_max;
  Eigen::VectorXd data_std_dev;
  Eigen::VectorXd data_std_dev_sqrd;
  Eigen::VectorXd data_std_dev_sum;

  Eigen::VectorXd mu_sum;
  Eigen::MatrixXd sig_sum;

  double error_init = 1000.0;
  double tol = 1e-4;
  double error = 1000;
  double llh_init = 0;
  double llh_prev_init = 0;
  double num_iter = 100;
  double svd_tol = 1e-4;// Tolerance for the SVD when finding the pseudoinverse and the determinant

  GMMFit();
  GMMFit(const std::vector<Eigen::VectorXd> & data_in, const int & num_clus_in);

  ~GMMFit();



  void setData(const std::vector<Eigen::VectorXd> & data_in); // This function lets you input a list of datums yourself, it does not work with the normalization routine (so don't use this one if you can avoid it)
  double multivariateGuassian(Eigen::VectorXd & x, Eigen::VectorXd & mu, Eigen::MatrixXd & Sigma); //This function is just the multivariate gaussian calc.
  double multivariateGuassian(Eigen::VectorXd & x, int cluster_index);

  void expectStep(); // expect step of the EM alg
  void setIter(const int & iter_in);
  void setTol(const double & tol_in);
  void setSVDTol(const double & svd_tol_in); // Sets the singular value threshold to use when computing the pseudo inverse and determinant
  void maxStep(); // maximization step of the EM alg
  double logLike(); // calculates the log likelihood for the EM alg
  void expectationMax(); // runs the full EM alg after being given data
  void setDim(const int & dim_in); // Sets the dimension of the problem, must be run in the beginning
  void setNumClusters(const int & num_clus_in); // sets the number of clusters to be found or input, must be run in the beginning
  void setMu(const std::vector<Eigen::VectorXd> & list_of_mus_in); // allows you to input a list of mus if you already have a trained model
  void setSigma(const std::vector<Eigen::MatrixXd> & list_of_Sigmas_in); // allows you to input a list of sigmas if you already have a trained model
  void setAlpha(const Eigen::VectorXd & alphs_in); // allows you to input an eigen vector of weights if you already have a trained model
  void randInitialGuess(); // initializes the random guess for the initial means of the clusters between -1 and 1
  void addData(Eigen::VectorXd & datum); // adds one single datum and adds it to the normalization variables
  void prepData(); // calculates the norm and the standard deviation of the data
  void initializeNormalization(); //Initializes normalization variables. Run after setting dimension
  void normalizeData(); // normalizes the data after it has been added using addData and you have run prepData
  void useRawData(); // lets you use the data without normalization
  double mixtureModelProb(Eigen::VectorXd & x_in); // given a particular state vector, outputs its "probability" given a mixture model.
  void normalizeInputCalculate(const Eigen::VectorXd & x_in, Eigen::VectorXd & x_normalized); // lets you input an unnormalized vector and you get a normalized vector
  void setDataParams(const Eigen::VectorXd & mean_in, const Eigen::VectorXd & std_dev_in); // allows you to input the data norm and standard deviation
  void normalizeInputInverse(const Eigen::VectorXd & x_in, Eigen::VectorXd & x_unnormalized); // allows you to input a normalized datum and it returns the unnormalized vector
};

#endif
