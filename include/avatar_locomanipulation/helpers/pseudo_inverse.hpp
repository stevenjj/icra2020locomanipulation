#ifndef ALM_PSEUDOINVERSE
#define ALM_PSEUDOINVERSE

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace math_utils{ 
  void pseudoInverse(const Eigen::MatrixXd & A,
                     Eigen::MatrixXd & Apinv,
                     double tolerance,
                     unsigned int computationOptions = Eigen::ComputeThinU | Eigen::ComputeThinV);

  void pseudoInverse(const Eigen::MatrixXd & A,
                     Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                     Eigen::MatrixXd & Apinv,
                     double tolerance,
                     unsigned int computationOptions);
}
#endif
