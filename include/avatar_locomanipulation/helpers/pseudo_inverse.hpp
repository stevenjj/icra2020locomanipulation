#ifndef ALM_PSEUDOINVERSE
#define ALM_PSEUDOINVERSE

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>

namespace math_utils{ 
  void pseudoInverse(const Eigen::MatrixXd & A,
                     Eigen::MatrixXd & Apinv,
                     double tolerance,
                     unsigned int computationOptions = Eigen::ComputeThinU | Eigen::ComputeThinV);

  void pseudoInverse(const Eigen::MatrixXd & A,
                     Eigen::MatrixXd & Apinv,
                     std::vector<double> & singular_values,
                     double tolerance,
                     unsigned int computationOptions = Eigen::ComputeThinU | Eigen::ComputeThinV);

  void pseudoInverse(const Eigen::MatrixXd & A,
                     Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                     Eigen::MatrixXd & Apinv,
                     double tolerance,
                     unsigned int computationOptions = Eigen::ComputeThinU | Eigen::ComputeThinV);

  void weightedPseudoInverse(const Eigen::MatrixXd & J, const Eigen::MatrixXd & Winv,
                             Eigen::MatrixXd & Jinv, double tolerance);
  
  void weightedPseudoInverse(const Eigen::MatrixXd & J, const Eigen::MatrixXd & Winv,
                          Eigen::JacobiSVD<Eigen::MatrixXd> & svdDecomposition,
                          Eigen::MatrixXd & Jinv, 
                          double tolerance);

}
#endif
