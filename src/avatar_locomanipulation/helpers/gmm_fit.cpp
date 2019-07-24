#include <avatar_locomanipulation/helpers/gmm_fit.hpp>

GMMFit::GMMFit(){
  // std::cout << "hello!" << std::endl;
}

GMMFit::~GMMFit(){
}

GMMFit::GMMFit(const std::vector<Eigen::VectorXd> & data_in, const int & num_clus_in){
 // setData(data_in);
 // setNumClusters(num_clus_in);
 std::cout << "this feature does not work" << std::endl;
}

void GMMFit::initializeNormalization() {
  data_mean = Eigen::VectorXd::Zero(dim);
  data_mean_sum = Eigen::VectorXd::Zero(dim);
  data_std_dev = Eigen::VectorXd::Zero(dim);
  data_std_dev_sum = Eigen::VectorXd::Zero(dim);
  data_std_dev_sqrd = Eigen::VectorXd::Zero(dim);
  data_min = Eigen::VectorXd::Constant(dim,1000.0);
  data_max = Eigen::VectorXd::Constant(dim,-1000.0);
}

void GMMFit::setData(const std::vector<Eigen::VectorXd> & data_in){
  num_data = data_in.size();
  // std::cout << "num_data: " << num_data << std::endl;
  dim = data_in[0].size();
  // std::cout << "dim: " << dim << std::endl;

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

  // std::cout << "sigma inverse: " << Sigma.inverse() << std::endl;

  x_mu = x-mu;
  detSig = Sigma.determinant();
  // std::cout << "detSigma: " << detSig << std::endl;
  den = pow(2.0*pi,dim/2.0)*pow(detSig,0.5);
  // std::cout << "den: " << den << std::endl;
  p = exp(-0.5*x_mu.transpose() * Sigma.inverse() * x_mu)/den;
  return p;
}

void GMMFit::expectStep(){
  double den;
  for(std::size_t i=0; i<num_data; i++) {
    den = 0.0;
    // std::cout << "2" << std::endl;
    for(std::size_t k=0; k<num_clus; k++){
      den = den+alphs[k]*multivariateGuassian(list_of_datums[i], list_of_mus[k], list_of_Sigmas[k]);
    }
    // std::cout << "3: " << den << std::endl;
    for(std::size_t k=0; k<num_clus; k++){
      // std::cout << "p: " << multivariateGuassian(list_of_datums[i], list_of_mus[k], list_of_Sigmas[k]) << std::endl;
      // std::cout << "alpha: " << alphs[k] << std::endl;
      gam(i,k) = alphs[k]*multivariateGuassian(list_of_datums[i], list_of_mus[k], list_of_Sigmas[k])/den;
      // std::cout << "4" << std::endl;
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
  }  Eigen::VectorXd data_mean = Eigen::VectorXd::Zero(dim);
  return llh;
}

void GMMFit::expectationMax(){
  double error = 1000.0;
  double tol = 1e-4;
  double llh = 0;
  double llh_prev = 0;
  int iter = 0;
  llh = logLike();
  // std::cout << "1" << std::endl;
  while (std::norm(error)>tol && iter<100){
    iter++;
    llh_prev = llh;
    expectStep();
    // std::cout << "5" << std::endl;
    maxStep();
    // std::cout << "6" << std::endl;
    llh = logLike();
    error = llh-llh_prev;
    std::cout << iter << " error: " << error << std::endl;
  }
  std::cout << "final error: " << error << std::endl;
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
  num_data = list_of_datums.size();
  gam = Eigen::MatrixXd::Zero(num_data, num_clus);
  n = Eigen::VectorXd::Zero(num_clus);
  mu_sum = Eigen::VectorXd::Zero(dim);
  sig_sum = Eigen::MatrixXd::Zero(dim, dim);
  alphs = Eigen::VectorXd::Constant(num_clus, 1.0/num_clus);
  for(std::size_t k=0; k<num_clus; k++){
    list_of_mus.push_back(Eigen::VectorXd::Random(dim));
    list_of_Sigmas.push_back(Eigen::MatrixXd::Identity(dim, dim));
  }
}

void GMMFit::addData(Eigen::VectorXd & datum){
  list_of_datums_raw.push_back(datum);
  data_mean_sum += datum;

  for(int i = 0; i<dim; i++){
    if (datum[i]<data_min[i]){
      data_min[i] = datum[i];
    }
    if (datum[i]>data_max[i]){
      data_max[i] = datum[i];
    }
  }

  // std::cout << data_min.transpose() << std::endl;
  // std::cout << data_max.transpose() << std::endl;
}

void GMMFit::prepData(){
  num_data = list_of_datums_raw.size();
  data_mean = data_mean_sum/(num_data);
  for (int i = 0; i<num_data; i++){
    for (int k = 0; k<dim; k++){
      data_std_dev_sum[k]+= pow(list_of_datums_raw[i][k]-data_mean[k],2);
    }
  }
  data_std_dev_sqrd = data_std_dev_sum/num_data;

  for (int k = 0; k<dim; k++){
    data_std_dev[k] = pow(data_std_dev_sqrd[k], .5);
  }
}

void GMMFit::normalizeData(){
  list_of_datums.clear();
  for(int i = 0; i<num_data; i++){
    list_of_datums.push_back((list_of_datums_raw[i]-data_mean).cwiseQuotient(data_std_dev));
  }
  // std::cout << list_of_datums.size() << " " << num_data << std::endl;
}

void GMMFit::useRawData(){
  list_of_datums = list_of_datums_raw;
}

double GMMFit::mixtureModelProb(Eigen::VectorXd & x_in){
  double p = 0;
  for(int i=0; i<num_clus; i++){
    p += alphs[i]*multivariateGuassian(x_in, list_of_mus[i], list_of_Sigmas[i]);
    // std::cout << "x: " << x_in << std::endl;
    // std::cout << "mu: " << list_of_mus[i] << std::endl;
    // std::cout << "sigma: " << list_of_Sigmas[i] << std::endl;
    // std::cout << "alpha: " << alphs[i] << std::endl;
    // std::cout << "p: " << p << std::endl;
    // std::cout << "--------------------------" << std::endl;
  }
  return p;
}

void GMMFit::normalizeInputCalculate(const Eigen::VectorXd & x_in, Eigen::VectorXd & x_normalized) {
  x_normalized = (x_in - data_mean).cwiseQuotient(data_std_dev);
}

void GMMFit::setDataParams(const Eigen::VectorXd & mean_in, const Eigen::VectorXd & std_dev_in) {
  data_mean = mean_in;
  data_std_dev = std_dev_in;
}

void GMMFit::normalizeInputInverse(const Eigen::VectorXd & x_in, Eigen::VectorXd & x_unnormalized) {
  x_unnormalized = (x_in.cwiseProduct(data_std_dev)) + data_mean;
}
