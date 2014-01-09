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

#ifndef ASSERT_MACROS_EIGEN_HPP_
#define ASSERT_MACROS_EIGEN_HPP_

#include <cmath>
#include "assert_macros.hpp"
#include <Eigen/Core>

#include "common.hpp"


namespace kindr {
namespace common {
namespace eigen {




inline bool compareRelative(double a, double b, double percentTolerance, double * percentError = NULL)
{
  // \todo: does anyone have a better idea?
  double fa = fabs(a);
  double fb = fabs(b);
  if( (fa < 1e-15 && fb < 1e-15) ||  // Both zero.
      (fa == 0.0  && fb < 1e-6)  ||  // One exactly zero and the other small
      (fb == 0.0  && fa < 1e-6) )    // ditto
    return true;

  double diff = fabs(a - b)/std::max(fa,fb);
  if(diff > percentTolerance * 1e-2)
  {
    if(percentError)
      *percentError = diff * 100.0;
    return false;
  }
  return true;
}


inline bool compareRelativePeriodic(double a, double b, double periodlength, double percentTolerance, double * percentError = NULL)
{
  // \todo: does anyone have a better idea?
  double fa = floatingPointModulo(a, periodlength); // a now lies in [0,periodlength)
  double fb = floatingPointModulo(b, periodlength); // b now lies in [0,periodlength)
  if( ((periodlength - 1e-15 < fa || fa < 1e-15) && (periodlength - 1e-15 < fb || fb < 1e-15)) ||  // Both zero or near periodlength
      (fa == 0.0  && fb < 1e-6)                                                                ||  // One exactly zero and the other small
      (fb == 0.0  && fa < 1e-6) )                                                                  // ditto
    return true;

  double diff = std::min(floatingPointModulo(a - b, periodlength), floatingPointModulo(b - a, periodlength))/periodlength;
  if(diff > percentTolerance * 1e-2)
  {
    if(percentError)
      *percentError = diff * 100.0;
    return false;
  }
  return true;
}


// kindr::common::eigen::assertNear((A),(B),1e-6,(MESSAGE))
/*          std::cout << MESSAGE << std::endl << "Difference between " << #A << " and " << #B << " exceeds tolerance of " << TOL << "." << std::endl; \
          std::cout << #A << "(" << i << "," << j << ") = " << (A)(i,j) << std::endl; \
          std::cout << #B << "(" << i << "," << j << ") = " << (B)(i,j) << std::endl; \ */
/*    std::cout << MESSAGE << std::endl << "Matrix " << #A << std::endl << A << std::endl << "and matrix " << #B << std::endl << B << std::endl << "have different sizes."; \ */

#ifdef NDEBUG
#define KINDR_ASSERT_MATRIX_NEAR_DBG(exceptionType, A, B, PERCENT_TOLERANCE, MSG)
#define KINDR_ASSERT_SCALAR_NEAR_DBG(exceptionType, A, B, PERCENT_TOLERANCE, MESSAGE)
#define PRINT(MESSAGE)
#else
#define KINDR_ASSERT_MATRIX_NEAR_DBG(exceptionType, A, B, PERCENT_TOLERANCE, MSG)       \
    if ((size_t)(A).rows() != (size_t)(B).rows()) { \
      std::stringstream kindr_assert_stringstream;  \
      kindr_assert_stringstream << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
      kindr::common::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    } \
    if ((size_t)(A).cols() != (size_t)(B).cols()) { \
      std::stringstream kindr_assert_stringstream;  \
      kindr_assert_stringstream << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
      kindr::common::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    } \
    for(int r = 0; r < (A).rows(); r++)                 \
    {                                 \
      for(int c = 0; c < (A).cols(); c++)               \
      {                               \
        double percentError = 0.0;                  \
        if(!kindr::common::eigen::compareRelative( (A)(r,c), (B)(r,c), PERCENT_TOLERANCE, &percentError)) { \
          std::stringstream kindr_assert_stringstream;  \
          kindr_assert_stringstream << MSG << "\nComparing:\n"                \
          << #A << "(" << r << "," << c << ") = " << (A)(r,c) << std::endl \
          << #B << "(" << r << "," << c << ") = " << (B)(r,c) << std::endl \
          << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n" \
          << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B; \
          kindr::common::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
        } \
      } \
    }
#define KINDR_ASSERT_SCALAR_NEAR_DBG(exceptionType, A, B, PERCENT_TOLERANCE, MESSAGE) \
    double percentError = 0.0; \
    if(!kindr::common::eigen::compareRelative( (A), (B), PERCENT_TOLERANCE, &percentError)) \
    { \
      std::stringstream kindr_assert_stringstream;  \
      kindr_assert_stringstream << MESSAGE << "\nComparing Scalars:\n"  \
      << "Scalar 1: " << #A << " = " << (A) << std::endl \
      << "Scalar 2: " << #B << " = " << (B) << std::endl \
      << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n"; \
      kindr::common::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
    }
#define PRINT(MESSAGE) std::cout << MESSAGE << std::endl;
#endif




#define KINDR_ASSERT_MAT_IS_FINITE(exceptionType, matrix, message)   \
    {                 \
  for(int r = 0; r < matrix.rows(); ++r)        \
  {                 \
    for(int c = 0; c < matrix.cols(); ++c)        \
    {               \
      if(!std::isfinite(matrix(r,c)))       \
      {               \
        std::stringstream kindr_assert_stringstream;   \
        kindr_assert_stringstream << "debug assert( isfinite(" << #matrix << "(" << r << ", " << c << ") ) failed. [ isfinite(" << matrix(r,c) << " ) ]" << message << std::endl << matrix; \
        kindr::common::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
      }               \
    }               \
  }                  \
    }


#ifndef NDEBUG

#define KINDR_ASSERT_MAT_IS_FINITE_DBG(exceptionType, matrix, message)   \
    {                 \
  for(int r = 0; r < matrix.rows(); ++r)        \
  {                 \
    for(int c = 0; c < matrix.cols(); ++c)        \
    {               \
      if(!std::isfinite(matrix(r,c)))       \
      {               \
        std::stringstream kindr_assert_stringstream;   \
        kindr_assert_stringstream << "assert( isfinite(" << #matrix << "(" << r << ", " << c << ") ) failed. [ isfinite(" << matrix(r,c) << " ) ]" << message << std::endl << matrix; \
        kindr::common::internal::kindr_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,kindr_assert_stringstream.str()); \
      }               \
    }               \
  }                  \
    }


#else

#define KINDR_ASSERT_MAT_IS_FINITE_DBG(exceptionType, matrix, message)

#endif



} // namespace eigen
} // namespace common
} // namespace rm



#endif /* ASSERT_MACROS_EIGEN_HPP_ */
