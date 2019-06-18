#include <avatar_locomanipulation/helpers/gmm_fit.hpp>

GMMFit::GMMFit(){
  // std::cout << "hello!" << std::endl;
  dim = 2;
}

GMMFit::~GMMFit(){
}

GMMFit::GMMFit(const Eigen::MatrixXd & data_in){
  data = data_in;
}

void GMMFit::setData(const std::vector<Eigen::VectorXd> & data_in){
  for(size_t i=0; i<data_in.size(); i++){
    data.col(i) = data_in[i];
  }
}

double GMMFit::multivariateGuassian(Eigen::VectorXd & x, Eigen::VectorXd & mu, Eigen::MatrixXd & Sigma){
  double p;
  double den;
  double detSig;
  Eigen::VectorXd x_mu;

  x_mu = x-mu;
  detSig = Sigma.determinant();
  den = pow(2*pi,dim/2)*pow(detSig,0.5);
  p = exp(-0.5*x_mu.transpose() * Sigma.inverse() * x_mu)/den;
  return p;
}
