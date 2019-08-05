// Code stripped from
// https://github.com/stack-of-tasks/tsid/blob/master/src/math/utils.cpp
#include <avatar_locomanipulation/helpers/pseudo_inverse.hpp>

namespace math_utils{ 
  void pseudoInverse(const Eigen::MatrixXd & A,
                     Eigen::MatrixXd & Apinv,
                     double tolerance,
                     unsigned int computationOptions){
    Eigen::JacobiSVD<Eigen::MatrixXd> svdDecomposition(A.rows(), A.cols());
    pseudoInverse(A, svdDecomposition, Apinv, tolerance, computationOptions);
  }
  



  void pseudoInverse(const Eigen::MatrixXd & A,
                     Eigen::MatrixXd & Apinv,
                     std::vector<double> & singular_values,
                     double tolerance,
                     unsigned int computationOptions){
    Eigen::JacobiSVD<Eigen::MatrixXd> svdDecomposition(A.rows(), A.cols());
    pseudoInverse(A, svdDecomposition, Apinv, tolerance, computationOptions);    
    // Also return the truncated singular values:
    using namespace Eigen;
    JacobiSVD<MatrixXd>::SingularValuesType singularValues = svdDecomposition.singularValues();
    long int singularValuesSize = singularValues.size();
    singular_values.clear(); // also store singular values
    for (long int idx = 0; idx < singularValuesSize; idx++) {
      singular_values.push_back(singularValues(idx));
    }

  }

  void pseudoInverse(const Eigen::MatrixXd & A,
                     Eigen::JacobiSVD<Eigen::MatrixXd> & svdDecomposition,
                     Eigen::MatrixXd & Apinv,
                     double tolerance,
                     unsigned int computationOptions){
    using namespace Eigen;
    
    if (computationOptions == 0) return; //if no computation options we cannot compute the pseudo inverse
    svdDecomposition.compute(A, computationOptions);
    
    JacobiSVD<MatrixXd>::SingularValuesType singularValues = svdDecomposition.singularValues();
    long int singularValuesSize = singularValues.size();
    int rank = 0;
    for (long int idx = 0; idx < singularValuesSize; idx++) {
      if (tolerance > 0 && singularValues(idx) > tolerance) {
        singularValues(idx) = 1.0 / singularValues(idx);
        rank++;
      } else {
        singularValues(idx) = 0.0;
      }
    }
    
    //equivalent to this U/V matrix in case they are computed full
    Apinv = svdDecomposition.matrixV().leftCols(singularValuesSize) * singularValues.asDiagonal() * svdDecomposition.matrixU().leftCols(singularValuesSize).adjoint();     
  }   


 
  void weightedPseudoInverse(const Eigen::MatrixXd & J, const Eigen::MatrixXd & Winv,
                             Eigen::MatrixXd & Jinv, double tolerance){
      Eigen::MatrixXd lambda(J* Winv * J.transpose());
      Eigen::MatrixXd lambda_inv;
      pseudoInverse(lambda, lambda_inv, tolerance);
      Jinv = Winv * J.transpose() * lambda_inv;
  }


   void weightedPseudoInverse(const Eigen::MatrixXd & J, const Eigen::MatrixXd & Winv,
                            Eigen::JacobiSVD<Eigen::MatrixXd> & svdDecomposition,
                            Eigen::MatrixXd & Jinv, 
                            double tolerance){
      Eigen::MatrixXd lambda(J* Winv * J.transpose());
      Eigen::MatrixXd lambda_inv;
      pseudoInverse(lambda, svdDecomposition, lambda_inv, tolerance);
      Jinv = Winv * J.transpose() * lambda_inv;
  }


}