/*!
* @file     Common.hpp
* @author   Remo Diethelm
* @date     22 10, 2013
* @version  1.0
* @ingroup  rm
* @brief
*/

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <cmath>
#include <cassert>
#include <iostream>

#include <sm/eigen/gtest.hpp>

//#include <Eigen/Core>
//#include <Eigen/Geometry>

namespace rm {
namespace common {



template<typename T>
T Mod(const T& x, const T& y)
{
    static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");

    if (y == 0.0)
        return x;

    double m= x - y * floor(x/y);

    // handle boundary cases resulted from floating-point cut off:

    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;

        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , 2*M_PI ): m= -1.421e-14
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -2*M_PI): m= 1.421e-14
        }
    }

    return m;
}

// wrap angle to [x1..x2)
template<typename T>
inline T wrapAngle(const T& angle, const T& x1, const T& x2)
{
    return Mod(angle-x1, x2-x1) + x1;
}

// wrap angle to [-PI..PI)
template<typename T>
inline T wrapPosNegPI(const T& angle)
{
    return Mod(angle + M_PI, 2*M_PI) - M_PI;
}

// wrap angle to [0..2*PI)
template<typename T>
inline T wrapTwoPI(const T& angle)
{
    return Mod(angle, 2*M_PI);
}


namespace eigen_implementation {

// sm::eigen::assertNear((A),(B),1e-6,(MESSAGE))
/*          std::cout << MESSAGE << std::endl << "Difference between " << #A << " and " << #B << " exceeds tolerance of " << TOL << "." << std::endl; \
          std::cout << #A << "(" << i << "," << j << ") = " << (A)(i,j) << std::endl; \
          std::cout << #B << "(" << i << "," << j << ") = " << (B)(i,j) << std::endl; \ */
/*    std::cout << MESSAGE << std::endl << "Matrix " << #A << std::endl << A << std::endl << "and matrix " << #B << std::endl << B << std::endl << "have different sizes."; \ */

#ifdef NDEBUG
#define ASSERT_MATRIX_NEAR(A, B, TOL, MESSAGE)
#define ASSERT_SCALAR_NEAR(A, B, TOL, MESSAGE)
#else
#define ASSERT_MATRIX_NEAR(A, B, TOL, MESSAGE) \
  if((A).rows() == (B).rows() && (A).cols() == (B).cols()) \
  { \
    for(int i=0; i<(A).rows(); i++) \
    { \
      for(int j=0; j<(B).rows(); j++) \
      { \
        if(abs((A)(i,j)-(B)(i,j)) > TOL) \
        { \
          std::cout << MESSAGE << std::endl; \
          std::cout << "Difference between the two matrices exceeds tolerance." << std::endl; \
          std::cout << "Assertion in file " << __FILE__ << ", line " << __LINE__ << "." << std::endl; \
          std::cout << "Matrix1(" << i << "," << j << ") = " << (A)(i,j) << std::endl; \
          std::cout << "Matrix2(" << i << "," << j << ") = " << (B)(i,j) << std::endl; \
          std::cout << (A) << std::endl; \
          std::cout << (B) << std::endl; \
          exit(-1); \
        } \
      } \
    } \
  } \
  else \
  { \
    std::cout << MESSAGE << std::endl; \
    std::cout << "The two matrices have different sizes."; \
    exit(-1); \
  }
#define ASSERT_SCALAR_NEAR(A, B, TOL, MESSAGE) \
  if(abs((A)-(B)) > TOL) \
  { \
    std::cout << MESSAGE << std::endl; \
    std::cout << "Difference between the two scalars exceeds tolerance." << std::endl; \
    std::cout << "Assertion in file " << __FILE__ << ", line " << __LINE__ << "." << std::endl; \
    std::cout << "Scalar1 = " << (A) << std::endl; \
    std::cout << "Scalar2 = " << (B) << std::endl; \
    exit(-1); \
  }
#endif

} // namespace eigen_implementation
} // namespace common
} // namespace rm



#endif /* COMMON_HPP_ */
