#include <avatar_locomanipulation/helpers/gmm_fit.hpp>


int main(int argc, char **argv){
  std::cout << "Testing GMM fit" << std::endl;
  GMMFit gmm_fit_object;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd mu = Eigen::VectorXd::Zero(2);
  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(2,2);

  double detSig;

  x[0] = 11;
  x[1] = 4;

  mu[0] = 10;
  mu[1] = 7;

  Sigma(0, 0) = 1;
  Sigma(0, 1) = 0.65;
  Sigma(1, 0) = 0.65;
  Sigma(1, 1) = 1;

  double p = gmm_fit_object.multivariateGuassian(x,mu,Sigma);
  std::cout << "probability is " << p << std::endl;
  return 0;
}
