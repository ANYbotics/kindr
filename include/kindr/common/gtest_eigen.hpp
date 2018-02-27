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

#include <type_traits>
#include <gtest/gtest.h>
#include "source_file_pos.hpp"
#include "assert_macros_eigen.hpp"
#include <Eigen/Core>

namespace kindr {

//TODO we should have a better place for test tools - especially libeigen independent ones
template <typename T>
double calcRotationQuatDisparityAngleToIdentity(const T & quat){
  // We need the shortest angle to 1 or -1, as the both represent rotation identity.
  // If the real part is nonnegative we can measure to 1 otherwise we should measure to -1.
  // The second case will have the same distance to -1 as the the negated quaternion to 1.
  // This means we can just look at the absolute value of the real part (the value we calculate the distance based on).
  const double wAbs = fabs(quat.w());
  // by a simple geometric intuition we get the distance to -1 in the unit quaternions (along a great circle) as acos(wAbs).
  // As the distance in rotations is always twice as big we finally get:
  return acos(wAbs)*2;
}

template <typename T>
double calcRotationQuatDisparityAngle(const T & q1, const T & q2){
  return calcRotationQuatDisparityAngleToIdentity(q1.inverted() * q2);
}



template<typename M1, typename M2>
void assertEqual(const M1 & A, const M2 & B, kindr::internal::source_file_pos const & sfp, std::string const & message = "")
{
  ASSERT_EQ((size_t)A.rows(),(size_t)B.rows()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n" << sfp.toString();
  ASSERT_EQ((size_t)A.cols(),(size_t)B.cols()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n" << sfp.toString();

  for(int r = 0; r < A.rows(); r++)
  {
    for(int c = 0; c < A.cols(); c++)
    {
      ASSERT_EQ(A(r,c),B(r,c)) << message << "\nEquality comparison failed at (" << r << "," << c << ")\n" << sfp.toString()
                                           << "\nMatrix A:\n" << A << "\nand matrix B\n" << B;
    }
  }
}


template<typename M1, typename M2, typename T>
void assertNear(const M1 & A, const M2 & B, T tolerance, kindr::internal::source_file_pos const & sfp, std::string const & message = "")
{
  // Note: If these assertions fail, they only abort this subroutine.
  // see: http://code.google.com/p/googletest/wiki/AdvancedGuide#Using_Assertions_in_Sub-routines
  // \todo better handling of this
  ASSERT_EQ((size_t)A.rows(),(size_t)B.rows()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n" << sfp.toString();
  ASSERT_EQ((size_t)A.cols(),(size_t)B.cols()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n" << sfp.toString();

  for(int r = 0; r < A.rows(); r++)
  {
    for(int c = 0; c < A.cols(); c++)
    {
      ASSERT_NEAR(A(r,c),B(r,c),tolerance) << message << "\nTolerance comparison failed at (" << r << "," << c << ")\n" << sfp.toString()
                               << "\nMatrix A:\n" << A << "\nand matrix B\n" << B;
    }
  }
}

template<typename M1, typename M2, typename T>
void expectNear(const M1 & A, const M2 & B, T tolerance, kindr::internal::source_file_pos const & sfp, std::string const & message = "")
{
  EXPECT_EQ((size_t)A.rows(),(size_t)B.rows()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n" << sfp.toString();
  EXPECT_EQ((size_t)A.cols(),(size_t)B.cols()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n" << sfp.toString();

  for(int r = 0; r < A.rows(); r++)
  {
    for(int c = 0; c < A.cols(); c++)
    {
      EXPECT_NEAR(A(r,c),B(r,c),tolerance) << message << "\nTolerance comparison failed at (" << r << "," << c << ")\n" << sfp.toString()
                               << "\nMatrix A:\n" << A << "\nand matrix B\n" << B;
    }
  }
}


template<typename M1>
void assertFinite(const M1 & A, kindr::internal::source_file_pos const & sfp, std::string const & message = "")
{
  for(int r = 0; r < A.rows(); r++)
  {
    for(int c = 0; c < A.cols(); c++)
    {
      ASSERT_TRUE(std::isfinite(A(r,c))) << sfp.toString() << std::endl << "Check for finite values failed at A(" << r << "," << c << "). Matrix A:" << std::endl << A << std::endl;
    }
  }
}

#define KINDR_ASSERT_DOUBLE_MX_EQ_ZT(A, B, PERCENT_TOLERANCE, MSG, ZERO_THRESHOLD)       \
    ASSERT_EQ((size_t)(A).rows(), (size_t)(B).rows())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    ASSERT_EQ((size_t)(A).cols(), (size_t)(B).cols())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    for(int r = 0; r < (A).rows(); r++)                 \
    {                                 \
      for(int c = 0; c < (A).cols(); c++)               \
      {                               \
        typedef typename std::remove_reference<decltype(A)>::type::Scalar Scalar; \
        Scalar percentError = static_cast<Scalar>(0.0); \
        ASSERT_TRUE(kindr::compareRelative( (A)(r,c), (B)(r,c), PERCENT_TOLERANCE, &percentError, ZERO_THRESHOLD)) \
        << MSG << "\nComparing:\n"                \
        << #A << "(" << r << "," << c << ") = " << (A)(r,c) << std::endl \
        << #B << "(" << r << "," << c << ") = " << (B)(r,c) << std::endl \
        << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n" \
        << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B; \
      }                               \
    }

#define KINDR_ASSERT_DOUBLE_MX_EQ(A, B, PERCENT_TOLERANCE, MSG)       \
    ASSERT_EQ((size_t)(A).rows(), (size_t)(B).rows())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    ASSERT_EQ((size_t)(A).cols(), (size_t)(B).cols())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    for(int r = 0; r < (A).rows(); r++)                 \
    {                                 \
      for(int c = 0; c < (A).cols(); c++)               \
      {                               \
        typedef typename std::remove_reference<decltype(A)>::type::Scalar Scalar; \
        Scalar percentError = static_cast<Scalar>(0.0); \
        ASSERT_TRUE(kindr::compareRelative( (A)(r,c), (B)(r,c), PERCENT_TOLERANCE, &percentError)) \
        << MSG << "\nComparing:\n"                \
        << #A << "(" << r << "," << c << ") = " << (A)(r,c) << std::endl \
        << #B << "(" << r << "," << c << ") = " << (B)(r,c) << std::endl \
        << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n" \
        << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B; \
      }                               \
    }

//! Matrix comparison with absolute and relative tolerance |M_MEAS - M_TRUE| <= |M_TRUE|*RTOL + ATOL
#define KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(M_TRUE, M_MEAS, ATOL, RTOL, MSG)       \
    ASSERT_EQ((size_t)(M_TRUE).rows(), (size_t)(M_MEAS).rows())  << MSG << "\nMatrix " << #M_TRUE << ":\n" << M_TRUE << "\nand matrix " << #M_MEAS << "\n" << M_MEAS << "\nare not the same size"; \
    ASSERT_EQ((size_t)(M_TRUE).cols(), (size_t)(M_MEAS).cols())  << MSG << "\nMatrix " << #M_TRUE << ":\n" << M_TRUE << "\nand matrix " << #M_MEAS << "\n" << M_MEAS << "\nare not the same size"; \
    for(int r = 0; r < (M_TRUE).rows(); r++)                 \
    {                                 \
      for(int c = 0; c < (M_TRUE).cols(); c++)               \
      {                               \
        typedef typename std::remove_reference<decltype(M_TRUE)>::type::Scalar Scalar; \
        ASSERT_TRUE(fabs((M_MEAS)(r,c) - (M_TRUE)(r,c)) <= (fabs((M_TRUE)(r,c))*RTOL + ATOL)) \
        << MSG << "\nComparing:\n"                \
        << #M_TRUE << "(" << r << "," << c << ") = " << (M_TRUE)(r,c) << std::endl \
        << #M_MEAS << "(" << r << "," << c << ") = " << (M_MEAS)(r,c) << std::endl \
        << "Absolute error was " << fabs((M_MEAS)(r,c) - (M_TRUE)(r,c)) << " > " <<  ATOL << "\n" \
        << "Relative error was " << fabs((M_MEAS)(r,c) - (M_TRUE)(r,c))/fabs((M_TRUE)(r,c)) << " > " << RTOL << "\n" \
        << "\nMatrix " << #M_TRUE << ":\n" << M_TRUE << "\nand matrix " << #M_MEAS << "\n" << M_MEAS; \
      }                               \
    }

#define KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(A, B, PERIODLENGTH, PERCENT_TOLERANCE, MSG) \
    ASSERT_EQ((size_t)(A).rows(), (size_t)(B).rows())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    ASSERT_EQ((size_t)(A).cols(), (size_t)(B).cols())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    for(int r = 0; r < (A).rows(); r++)                 \
    {                                 \
      for(int c = 0; c < (A).cols(); c++)               \
      {                               \
        typedef typename std::remove_reference<decltype(A)>::type::Scalar Scalar; \
        Scalar percentError = static_cast<Scalar>(0.0); \
        ASSERT_TRUE(kindr::compareRelativePeriodic( (A)(r,c), (B)(r,c), PERIODLENGTH, PERCENT_TOLERANCE, &percentError)) \
        << MSG << "\nComparing:\n"                \
        << #A << "(" << r << "," << c << ") = " << (A)(r,c) << std::endl \
        << #B << "(" << r << "," << c << ") = " << (B)(r,c) << std::endl \
        << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n" \
        << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B; \
      }                               \
    }




} // namespace kindr


