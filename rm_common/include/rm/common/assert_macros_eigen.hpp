/*
 * assert_macros_eigen.hpp
 *
 *  Created on: Nov 20, 2013
 *      Author: gech
 */

#ifndef ASSERT_MACROS_EIGEN_HPP_
#define ASSERT_MACROS_EIGEN_HPP_

#include <cmath>
#include "assert_macros.hpp"
#include <Eigen/Core>


namespace rm {
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


// rm::eigen::assertNear((A),(B),1e-6,(MESSAGE))
/*          std::cout << MESSAGE << std::endl << "Difference between " << #A << " and " << #B << " exceeds tolerance of " << TOL << "." << std::endl; \
          std::cout << #A << "(" << i << "," << j << ") = " << (A)(i,j) << std::endl; \
          std::cout << #B << "(" << i << "," << j << ") = " << (B)(i,j) << std::endl; \ */
/*    std::cout << MESSAGE << std::endl << "Matrix " << #A << std::endl << A << std::endl << "and matrix " << #B << std::endl << B << std::endl << "have different sizes."; \ */

#ifdef NDEBUG
#define RM_ASSERT_MATRIX_NEAR_DBG(exceptionType, A, B, PERCENT_TOLERANCE, MSG)
#define RM_ASSERT_SCALAR_NEAR_DBG(exceptionType, A, B, PERCENT_TOLERANCE, MESSAGE)
#define PRINT(MESSAGE)
#else
#define RM_ASSERT_MATRIX_NEAR_DBG(exceptionType, A, B, PERCENT_TOLERANCE, MSG)       \
    if ((size_t)(A).rows() != (size_t)(B).rows()) { \
      std::stringstream rm_assert_stringstream;  \
      rm_assert_stringstream << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
      rm::detail::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,rm_assert_stringstream.str()); \
    } \
    if ((size_t)(A).cols() != (size_t)(B).cols()) { \
      std::stringstream rm_assert_stringstream;  \
      rm_assert_stringstream << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
      rm::detail::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,rm_assert_stringstream.str()); \
    } \
    for(int r = 0; r < (A).rows(); r++)                 \
    {                                 \
      for(int c = 0; c < (A).cols(); c++)               \
      {                               \
        double percentError = 0.0;                  \
        if(!rm::eigen::compareRelative( (A)(r,c), (B)(r,c), PERCENT_TOLERANCE, &percentError)) { \
          std::stringstream rm_assert_stringstream;  \
          rm_assert_stringstream << MSG << "\nComparing:\n"                \
          << #A << "(" << r << "," << c << ") = " << (A)(r,c) << std::endl \
          << #B << "(" << r << "," << c << ") = " << (B)(r,c) << std::endl \
          << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n" \
          << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B; \
          rm::detail::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,rm_assert_stringstream.str()); \
        } \
      } \
    }
#define RM_ASSERT_SCALAR_NEAR_DBG(exceptionType, A, B, PERCENT_TOLERANCE, MESSAGE) \
  double percentError = 0.0; \
  if(!rm::eigen::compareRelative( (A), (B), PERCENT_TOLERANCE, &percentError)) \
  { \
    std::stringstream rm_assert_stringstream;  \
    rm_assert_stringstream << MESSAGE << "\nComparing Scalars:\n"  \
    << "Scalar 1: " << #A << " = " << (A) << std::endl \
    << "Scalar 2: " << #B << " = " << (B) << std::endl \
    << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n"; \
    rm::detail::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,rm_assert_stringstream.str()); \
  }
#define PRINT(MESSAGE) std::cout << MESSAGE << std::endl;
#endif




#define RM_ASSERT_MAT_IS_FINITE(exceptionType, matrix, message)   \
  {                 \
  for(int r = 0; r < matrix.rows(); ++r)        \
    {                 \
      for(int c = 0; c < matrix.cols(); ++c)        \
  {               \
    if(!std::isfinite(matrix(r,c)))       \
      {               \
        std::stringstream rm_assert_stringstream;   \
        rm_assert_stringstream << "debug assert( isfinite(" << #matrix << "(" << r << ", " << c << ") ) failed. [ isfinite(" << matrix(r,c) << " ) ]" << message << std::endl << matrix; \
        rm::detail::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,rm_assert_stringstream.str()); \
      }               \
  }               \
     }                  \
}


#ifndef NDEBUG

#define RM_ASSERT_MAT_IS_FINITE_DBG(exceptionType, matrix, message)   \
  {                 \
  for(int r = 0; r < matrix.rows(); ++r)        \
    {                 \
      for(int c = 0; c < matrix.cols(); ++c)        \
  {               \
    if(!std::isfinite(matrix(r,c)))       \
      {               \
        std::stringstream rm_assert_stringstream;   \
        rm_assert_stringstream << "assert( isfinite(" << #matrix << "(" << r << ", " << c << ") ) failed. [ isfinite(" << matrix(r,c) << " ) ]" << message << std::endl << matrix; \
        rm::detail::rm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,rm_assert_stringstream.str()); \
      }               \
  }               \
     }                  \
}


#else

#define RM_ASSERT_MAT_IS_FINITE_DBG(exceptionType, matrix, message)

#endif


} // namespace eigen
} // namespace rm



#endif /* ASSERT_MACROS_EIGEN_HPP_ */
