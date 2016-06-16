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

/*
 * gtest_rotations.hpp
 *
 *  Created on: Jun 16, 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <gtest/gtest.h>
#include <kindr/Core>
#include <kindr/common/gtest_eigen.hpp>

namespace kindr {


/*! Implement the conversions from the kindr convention to the other convention.
 *  Specialize the conversion traits
 */
template<typename OtherQuaternion_, typename OtherVelocity_, typename PrimType_>
class ConversionTraits {
 public:
  inline static bool convertQuaternion(OtherQuaternion_& quaternionOut, const kindr::RotationQuaternion<PrimType_>& quaternionIn) {
    // Implement quaternionOut = f(quaternionIn);
    return false;
  }

  inline static bool convertVelocityVector(OtherVelocity_& velocityOut, const OtherQuaternion_& rot, const Eigen::Matrix<PrimType_,3,1>& velocityIn) {
    // Implement velocityOut = g(velocityIn, rot);
    return false;
  }

  inline static bool getRotationMatrixFromQuaternion(Eigen::Matrix<PrimType_,3,3>& rotationMatrix, const OtherQuaternion_& quaternion) {
    // Implement rotationMatrix = C(quaternion);
    return false;
  }

  inline static bool boxPlus(OtherQuaternion_& res, const OtherQuaternion_& quaternion, const OtherVelocity_& velocity) {
    // Implement res = quternion.boxPlus(vector);
    return false;
  }

  inline static bool testQuaternion(const OtherQuaternion_& expected, const OtherQuaternion_& actual) {
    // Implement EXPECT_NEAR(expected, actual, 1.0e-6);
    return false;
  }
};

//! Convention tests that need to be fulfilled
/*! Implement the ConversionTraits and the following unit tests (RBDL example):
  TEST(ConventionTest, Concatenation) {
    ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testConcatenation();
  }
  TEST(ConventionTest, Rotation) {
    ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testRotation();
  }
  TEST(ConventionTest, BoxPlus) {
    ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testBoxPlus();
  }
 */
template<typename OtherQuaternion_, typename OtherVelocity_, typename PrimType_>
class ConventionTest {
 public:
  inline static void testConcatenation() {
    kindr::RotationQuaternion<PrimType_> kindrQuat1(kindr::EulerAnglesZyx<PrimType_>(0.1, 0.2, 0.3));
    kindr::RotationQuaternion<PrimType_> kindrQuat2(kindr::EulerAnglesZyx<PrimType_>(-0.4, 0.8, -0.5));
    OtherQuaternion_ otherQuat1;
    bool res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::convertQuaternion(otherQuat1, kindrQuat1);
    ASSERT_TRUE(res);
    OtherQuaternion_ otherQuat2;
    res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::convertQuaternion(otherQuat2, kindrQuat2);
    ASSERT_TRUE(res);

    kindr::RotationQuaternion<PrimType_> kindrQuatKindrConcat = kindrQuat1*kindrQuat2;
    OtherQuaternion_ otherQuatKindrConcat;
    res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::convertQuaternion(otherQuatKindrConcat, kindrQuatKindrConcat);
    ASSERT_TRUE(res);

    OtherQuaternion_ otherQuatOtherConcat = otherQuat1*otherQuat2;

    // Test quaternions
    res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::testQuaternion(otherQuatKindrConcat, otherQuatKindrConcat);
    ASSERT_TRUE(res);
  }

  inline static void testRotation() {
    kindr::RotationQuaternion<PrimType_> kindrQuat(kindr::EulerAnglesZyx<PrimType_>(-0.4, 0.8, -0.5));
    kindr::RotationMatrix<PrimType_> kindrMatrix(kindrQuat);
    OtherQuaternion_ otherQuat;
    bool res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::convertQuaternion(otherQuat, kindrQuat);
    ASSERT_TRUE(res);

    Eigen::Matrix3d eigenOtherMatrix;
    res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::getRotationMatrixFromQuaternion(eigenOtherMatrix, otherQuat);
    ASSERT_TRUE(res);

    Eigen::Matrix3d eigenKindrMatrix = kindrMatrix.matrix();

    // Test matrices
    KINDR_ASSERT_DOUBLE_MX_EQ(eigenKindrMatrix, eigenOtherMatrix, 1.0e-3, "rotation matrix");
  }

  inline static void testBoxPlus() {
    kindr::RotationQuaternion<PrimType_> kindrQuat(kindr::EulerAnglesZyx<PrimType_>(-0.4, 0.8, -0.5));
    Eigen::Matrix<PrimType_, 3, 1> kindrVelocity(0.5, 0.7, 0.9);
    OtherQuaternion_ otherQuat;

    bool res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::convertQuaternion(otherQuat, kindrQuat);
    ASSERT_TRUE(res);

    OtherVelocity_ otherVelocity;
    res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::convertVelocityVector(otherVelocity, otherQuat, kindrVelocity);
    ASSERT_TRUE(res);

    OtherQuaternion_ otherQuatPlus;
    res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::boxPlus(otherQuatPlus, otherQuat, otherVelocity);
    kindr::RotationQuaternion<PrimType_> kindrQuatPlus = kindrQuat.boxPlus(kindrVelocity);
    ASSERT_TRUE(res);

    OtherQuaternion_ otherQuatKindrPlus;
    res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::convertQuaternion(otherQuatKindrPlus, kindrQuatPlus);
    ASSERT_TRUE(res);

    // Test quaternions
    res = ConversionTraits<OtherQuaternion_, OtherVelocity_, PrimType_>::testQuaternion(otherQuatKindrPlus, otherQuatPlus);
    ASSERT_TRUE(res);
  }
};

} // namespace

