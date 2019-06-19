#include <avatar_locomanipulation/helpers/gmm_fit.hpp>

GMMFit::GMMFit(){
  // std::cout << "hello!" << std::endl;
}

GMMFit::~GMMFit(){
}

GMMFit::GMMFit(const std::vector<Eigen::VectorXd> & data_in, const int & num_clus_in){
 setData(data_in);
 setNumClusters(num_clus_in);
}

void GMMFit::setData(const std::vector<Eigen::VectorXd> & data_in){
  num_data = data_in.size();
  std::cout << "num_data: " << num_data << std::endl;
  dim = data_in[0].size();
  std::cout << "dim: " << dim << std::endl;

  gam = Eigen::MatrixXd::Zero(num_data, num_clus);
  n = Eigen::VectorXd::Zero(num_clus);

  list_of_datums=data_in;
  mu_sum = Eigen::VectorXd::Zero(dim);
  sig_sum = Eigen::MatrixXd::Zero(dim, dim);
}

double GMMFit::multivariateGuassian(Eigen::VectorXd & x, Eigen::VectorXd & mu, Eigen::MatrixXd & Sigma){
  double p;
  double den;
  double detSig;
  Eigen::VectorXd x_mu;

  x_mu = x-mu;
  detSig = Sigma.determinant();
  den = pow(2.0*pi,dim/2.0)*pow(detSig,0.5);
  p = exp(-0.5*x_mu.transpose() * Sigma.inverse() * x_mu)/den;
  return p;
}

void GMMFit::expectStep(){
  double den;
  for(std::size_t i=0; i<num_data; i++) {
    den = 0.0;
    for(std::size_t k=0; k<num_clus; k++){
      den = den+alphs[k]*multivariateGuassian(list_of_datums[i], list_of_mus[k], list_of_Sigmas[k]);
    }
    for(std::size_t k=0; k<num_clus; k++){
      gam(i,k) = alphs[k]*multivariateGuassian(list_of_datums[i], list_of_mus[k], list_of_Sigmas[k])/den;
    }
  }
}

void GMMFit::maxStep(){
  for(std::size_t k=0; k<num_clus; k++){
    n[k] = gam.col(k).sum();
    alphs[k] = n[k]/num_data;
  }

  for(std::size_t k=0; k<num_clus; k++){
    mu_sum.setZero();
    for(std::size_t i=0; i<num_data; i++){
      mu_sum = mu_sum + gam(i,k)*list_of_datums[i];
    }
    list_of_mus[k] = mu_sum/n[k];
  }

  for(std::size_t k=0; k<num_clus; k++){
    sig_sum.setZero();
    for(std::size_t i=0; i<num_data; i++){
      sig_sum = sig_sum + gam(i,k) * (list_of_datums[i]-list_of_mus[k])*((list_of_datums[i]-list_of_mus[k]).transpose());
    }
    list_of_Sigmas[k] = sig_sum/n[k];
  }
}

double GMMFit::logLike(){
  double llh = 0.0;
  double lh = 0.0;
  for(std::size_t i=0; i<num_data; i++){
    lh = 0.0;
    for(std::size_t k=0; k<num_clus; k++){
      lh = lh+alphs[k]*multivariateGuassian(list_of_datums[i], list_of_mus[k], list_of_Sigmas[k]);
    }
    llh = llh + log(lh);
  }
  return llh;
}

void GMMFit::expectationMax(){
  double error = 1000.0;
  double tol = 1e-4;
  double llh = 0;
  double llh_prev = 0;

  llh = logLike();
  while (std::norm(error)>tol){
    llh_prev = llh;
    expectStep();
    maxStep();
    llh = logLike();
    error = llh-llh_prev;
  }
}

void GMMFit::setDim(const int & dim_in){
  dim = dim_in;
}

void GMMFit::setNumClusters(const int & num_clus_in){
  num_clus = num_clus_in;
}

void GMMFit::setMu(const std::vector<Eigen::VectorXd> & list_of_mus_in){
  list_of_mus = list_of_mus_in;
}

void GMMFit::setSigma(const std::vector<Eigen::MatrixXd> & list_of_Sigmas_in){
  list_of_Sigmas = list_of_Sigmas_in;
}

void GMMFit::setAlpha(const Eigen::VectorXd & alphs_in){
  alphs = alphs_in;
}

void GMMFit::randInitialGuess(){
  alphs = Eigen::VectorXd::Constant(num_clus, 1.0/num_clus);
  for(std::size_t k=0; k<num_clus; k++){
    list_of_mus.push_back(Eigen::VectorXd::Random(dim));
    list_of_Sigmas.push_back(Eigen::MatrixXd::Identity(dim, dim));
  }
}
