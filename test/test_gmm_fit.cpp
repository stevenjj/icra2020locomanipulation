#include <avatar_locomanipulation/helpers/gmm_fit.hpp>


int main(int argc, char **argv){
  std::cout << "Testing GMM fit" << std::endl;
  GMMFit gmmfitter;

  // Test multivariateGuassian
  std::cout << "Testing multivariateGuassian" << std::endl;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd mu_one = Eigen::VectorXd::Zero(2);
  Eigen::MatrixXd Sigma_one = Eigen::MatrixXd::Zero(2,2);

  x[0] = 11;
  x[1] = 4;

  mu_one[0] = 10;
  mu_one[1] = 7;

  Sigma_one(0, 0) = 1;
  Sigma_one(0, 1) = 0.75;
  Sigma_one(1, 0) = 0.75;
  Sigma_one(1, 1) = 1;

  gmmfitter.setDim(2);
  double p = gmmfitter.multivariateGuassian(x,mu_one,Sigma_one);
  std::cout << "probability is " << p << std::endl;

  // Test logLike
  std::cout << "Testing logLike" << std::endl;
  Eigen::VectorXd mu_two = Eigen::VectorXd::Zero(2);
  Eigen::MatrixXd Sigma_two = Eigen::MatrixXd::Zero(2,2);

  mu_two[0] = 15;
  mu_two[1] = 10;

  Sigma_two(0, 0) = 1;
  Sigma_two(0, 1) = 0.75;
  Sigma_two(1, 0) = 0.75;
  Sigma_two(1, 1) = 1;

  std::vector<Eigen::VectorXd> list_of_mus;
  std::vector<Eigen::MatrixXd> list_of_Sigmas;
  std::vector<Eigen::VectorXd> list_of_datums;
  Eigen::VectorXd alphs;
  Eigen::VectorXd datum_one;
  Eigen::VectorXd datum_two;
  Eigen::VectorXd datum_three;
  Eigen::VectorXd datum_four;

  list_of_mus.push_back(mu_one);
  list_of_mus.push_back(mu_two);

  list_of_Sigmas.push_back(Sigma_one);
  list_of_Sigmas.push_back(Sigma_two);

  alphs = Eigen::VectorXd::Zero(2);
  alphs[0] = .7;
  alphs[1] = .3;

  datum_one = Eigen::VectorXd::Zero(2);
  datum_one[0] = 9.022100000000000;
  datum_one[1] = 6.318700000000000;

  datum_two = Eigen::VectorXd::Zero(2);
  datum_two[0] = 9.786899999999999;
  datum_two[1] = 7.236800000000000;

  datum_three = Eigen::VectorXd::Zero(2);
  datum_three[0] = 15.765000000000001;
  datum_three[1] = 11.498600000000000;

  datum_four = Eigen::VectorXd::Zero(2);
  datum_four[0] = 15.558700000000000;
  datum_four[1] = 10.454599999999999;

  list_of_datums.push_back(datum_one);
  list_of_datums.push_back(datum_two);
  list_of_datums.push_back(datum_three);
  list_of_datums.push_back(datum_four);

  gmmfitter.setNumClusters(2);
  gmmfitter.setMu(list_of_mus);
  gmmfitter.setSigma(list_of_Sigmas);
  gmmfitter.setAlpha(alphs);
  gmmfitter.setData(list_of_datums);

  double llh = gmmfitter.logLike();
  std::cout << "log likelihood is " << llh << std::endl;

  // Test expectStep
  std::cout << "Testing expectStep" << std::endl;
  gmmfitter.expectStep();
  std::cout << "gam: " << gmmfitter.gam << std::endl;

  // Test maxStep
  std::cout << "Testing maxStep" << std::endl;
  gmmfitter.maxStep();
  std::cout << "alph: " << gmmfitter.alphs << std::endl;
  for (size_t k=0; k<2; k++){
    std::cout << "mu " << k+1 << ": " << gmmfitter.list_of_mus[k] << std::endl;
    std::cout << "sig " << k+1 << ": " << gmmfitter.list_of_Sigmas[k] << std::endl;
  }

  //Test expectationMax

  mu_one[0] = 16;
  mu_one[1] = 12;

  Sigma_one(0, 0) = 1;
  Sigma_one(0, 1) = 0.75;
  Sigma_one(1, 0) = 0.75;
  Sigma_one(1, 1) = 1;

  mu_two[0] = 12;
  mu_two[1] = 9;

  Sigma_two(0, 0) = 1;
  Sigma_two(0, 1) = 0.75;
  Sigma_two(1, 0) = 0.75;
  Sigma_two(1, 1) = 1;

  list_of_mus[0] = mu_one;
  list_of_mus[1] = mu_two;

  list_of_Sigmas[0] = Sigma_one;
  list_of_Sigmas[1] = Sigma_two;

  gmmfitter.setMu(list_of_mus);
  gmmfitter.setSigma(list_of_Sigmas);
  gmmfitter.setAlpha(alphs);

  std::cout << "Testing expectationMax" << std::endl;
  gmmfitter.expectationMax();
  std::cout << "gam: " << gmmfitter.gam << std::endl;
  std::cout << "alph: " << gmmfitter.alphs << std::endl;
  for (size_t k=0; k<2; k++){
    std::cout << "mu " << k+1 << ": " << gmmfitter.list_of_mus[k] << std::endl;
    std::cout << "sig " << k+1 << ": " << gmmfitter.list_of_Sigmas[k] << std::endl;
  }


  return 0;
}
