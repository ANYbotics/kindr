/*
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#pragma once

#include <Eigen/SVD>

namespace kindr {


/*!
 * \brief Gets a skew-symmetric matrix from a (column) vector
 * \param   vec 3x1-matrix (column vector)
 * \return skew   3x3-matrix
 */
template<typename PrimType_>
inline static Eigen::Matrix<PrimType_, 3, 3> getSkewMatrixFromVector(const Eigen::Matrix<PrimType_, 3, 1>& vec) {
  Eigen::Matrix<PrimType_, 3, 3> mat;
  mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return mat;
}

/*!
 * \brief Gets a 3x1 vector from a skew-symmetric matrix
 * \param   matrix 3x3-matrix
 * \return  column vector (3x1-matrix)
 */
template<typename PrimType_>
inline static Eigen::Matrix<PrimType_, 3, 1> getVectorFromSkewMatrix(const Eigen::Matrix<PrimType_, 3, 3>& matrix) {
  return Eigen::Matrix<PrimType_, 3, 1> (matrix(2,1), matrix(0,2), matrix(1,0));
}

/*!
 * \brief Computes the Moore–Penrose pseudoinverse
 * info: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
 * \param a: Matrix to invert
 * \param result: Result is written here
 * \param epsilon: Numerical precision (for example 1e-6)
 * \return true if successful
 */
template<typename _Matrix_TypeA_, typename _Matrix_TypeB_>
bool static pseudoInverse(const _Matrix_TypeA_ &a, _Matrix_TypeB_ &result,
                          double epsilon = std::numeric_limits<typename _Matrix_TypeA_::Scalar>::epsilon())
{
  // Shorthands
  constexpr auto rowsA = static_cast<int>(_Matrix_TypeA_::RowsAtCompileTime);
  constexpr auto colsA = static_cast<int>(_Matrix_TypeA_::ColsAtCompileTime);
  constexpr auto rowsB = static_cast<int>(_Matrix_TypeB_::RowsAtCompileTime);
  constexpr auto colsB = static_cast<int>(_Matrix_TypeB_::ColsAtCompileTime);

  // Assert wrong matrix types
  static_assert(std::is_same<typename _Matrix_TypeA_::Scalar, typename _Matrix_TypeB_::Scalar>::value,
                "[kindr::pseudoInverse] Matrices must be of the same Scalar type!");
  static_assert(rowsA == colsB && colsA == rowsB, "[kindr::pseudoInverse] Result type has wrong size!");

  // If one dimension is dynamic, compute everything as dynamic size
  constexpr auto m = Eigen::JacobiSVD< _Matrix_TypeA_ >::DiagSizeAtCompileTime;

  // JacobiSVD needs to be computed on dynamic size if we need ComputeThinU, ComputeThinV, see:
  // https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html
  Eigen::JacobiSVD< Eigen::Matrix<typename _Matrix_TypeA_::Scalar, Eigen::Dynamic, Eigen::Dynamic> > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

  typename _Matrix_TypeA_::Scalar tolerance =
    epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

  // Sigma for ThinU and ThinV
  Eigen::Matrix<typename _Matrix_TypeA_::Scalar, m, m> sigmaThin = Eigen::Matrix<typename _Matrix_TypeA_::Scalar, m, 1>(
    (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal();

  result = svd.matrixV() * sigmaThin * svd.matrixU().transpose();

  return true;
}


} // end namespace kindr
