/*
 * LinearAlgebra.hpp
 *
 *  Created on: October 16, 2012
 *      Author: gech, Peter Fankhauser
 */

#ifndef RM_LINEARALGEBRA_HPP_
#define RMLINEARALGEBRA_HPP_

#include <Eigen/SVD>

namespace rm {
namespace linear_algebra {

/*! Gets a skew-symmetric matrix from a vector
 * @param vec vector
 * @return skew matrix
 */
Eigen::Matrix3d getSkewMatrixFromVector(const Eigen::Vector3d& vec);

/*! Computes the Moore–Penrose pseudoinverse
 * info: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
 * @param a: Matrix to invert
 * @param result: Result is written here
 * @param epsilon: Numerical precision (for example 1e-6)
 * @return true if successful
 */
template<typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
{
  Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

  result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();

  return true;
}

} // end namespace linear_algebra
} // end namespace rm

#endif /* RM_LINEARALGEBRA_HPP_ */
