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
#include <kindr/rotations/RotationConversion.hpp>

namespace kindr {

//! Convention tests that need to be fulfilled
/*! Implement the ConversionTraits and the following unit tests (RBDL example):
  TEST(ConventionTest, Concatenation) {
    ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testConcatenation();
  }
  TEST(ConventionTest, Rotation) {
    ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testRotationMatrix();
  }
  TEST(ConventionTest, BoxPlus) {
    ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testBoxPlus();
  }
 */
template<typename OtherRotation_, typename OtherVelocity_, typename PrimType_>
class ConventionTest {
 public:
  inline static void testConcatenation() {
    kindr::RotationQuaternion<PrimType_> kindrQuat1(kindr::EulerAnglesZyx<PrimType_>(0.1, 0.2, 0.3));
    kindr::RotationQuaternion<PrimType_> kindrQuat2(kindr::EulerAnglesZyx<PrimType_>(-0.4, 0.8, -0.5));
    OtherRotation_ otherQuat1;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::convertToOtherRotation(otherQuat1, kindrQuat1);

    OtherRotation_ otherQuat2;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::convertToOtherRotation(otherQuat2, kindrQuat2);

    // Concatenate with Kindr
    kindr::RotationQuaternion<PrimType_> kindrQuatKindrConcat = kindrQuat2*kindrQuat1;

    // Concatenate with other rotation
    OtherRotation_ otherQuatOtherConcat;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::concatenate(otherQuatOtherConcat, otherQuat1, otherQuat2);

    kindr::RotationQuaternion<PrimType_> kindrQuatOtherConcat;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::convertToKindr(kindrQuatOtherConcat, otherQuatOtherConcat);

    // Test quaternions
    ASSERT_TRUE(kindrQuatOtherConcat.isNear(kindrQuatKindrConcat, 1.0e-3));
  }

  inline static void testRotationMatrix() {
    kindr::RotationQuaternion<PrimType_> kindrQuat(kindr::EulerAnglesZyx<PrimType_>(-0.4, 0.8, -0.5));
    kindr::RotationMatrix<PrimType_> kindrMatrix(kindrQuat);
    OtherRotation_ otherQuat;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::convertToOtherRotation(otherQuat, kindrQuat);

    Eigen::Matrix<PrimType_, 3, 3> eigenOtherMatrix;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::getRotationMatrixFromRotation(eigenOtherMatrix, otherQuat);

    Eigen::Matrix<PrimType_, 3, 3> eigenKindrMatrix = kindrMatrix.matrix();

    // Test matrices
    KINDR_ASSERT_DOUBLE_MX_EQ(eigenKindrMatrix, eigenOtherMatrix, 1.0e-3, "rotation matrix");

  }


  inline static void testGeometricalInterpretation() {
    /* Consider two coordinate frames A and B. A is fixed, while B is rotated around the z-axis by alpha = pi/4 w.r.t. A.
     * The unit vectors of B in B frame are the vectors of the canonical base of R^3.
     * Using a geometrical approach, it can easily be shown that the components of these vectors expressed in A frame are:
     *
     *   A_r_Bx = [ cos(alpha) sin(alpha)  0]^T;
     *   A_r_By = [-sin(alpha) cos(alpha)  0]^T;
     *   A_r_Bz = [ 0          0           1]^T;
     *
     * Stacking these transformations column-wise returns a rotation matrix that projects the coordinates of a vector
     * expressed in the B frame to those of a vector expressed in A frame:
     *
     *  C_AB = [ A_r_Bx   A_r_By   A_r_Bz]
     *
     *  A_r = C_AB * B_r
     */
    using std::cos;
    using std::sin;
    typedef Eigen::Matrix<PrimType_, 3, 1> Vector;

    // Angle of elementary rotation
    const PrimType_ alpha = M_PI_4;
    kindr::RotationQuaternion<PrimType_> rotationBToA;

    Vector expected_A_r_Bx, expected_A_r_By, expected_A_r_Bz;

    // Elementary rotation around z-axis
    rotationBToA(kindr::EulerAnglesZyx<PrimType_>(alpha, 0.0, 0.0));
    expected_A_r_Bx = Vector(cos(alpha),     sin(alpha),     PrimType_(0.0));
    expected_A_r_By = Vector(-sin(alpha),    cos(alpha),     PrimType_(0.0));
    expected_A_r_Bz = Vector(PrimType_(0.0), PrimType_(0.0), PrimType_(1.0));
    checkGeometricalInterpretation(rotationBToA, expected_A_r_Bx, expected_A_r_By, expected_A_r_Bz);

    // Elementary rotation around y-axis
    rotationBToA(kindr::EulerAnglesZyx<PrimType_>(0.0, alpha, 0.0));
    expected_A_r_Bx = Vector(cos(alpha),     PrimType_(0.0), -sin(alpha));
    expected_A_r_By = Vector(PrimType_(0.0), PrimType_(1.0), PrimType_(0.0));
    expected_A_r_Bz = Vector(sin(alpha),     PrimType_(0.0), cos(alpha));
    checkGeometricalInterpretation(rotationBToA, expected_A_r_Bx, expected_A_r_By, expected_A_r_Bz);

    // Elementary rotation around x-axis
    rotationBToA(kindr::EulerAnglesZyx<PrimType_>(0.0, 0.0, alpha));
    expected_A_r_Bx = Vector(PrimType_(1.0), PrimType_(0.0), PrimType_(0.0));
    expected_A_r_By = Vector(PrimType_(0.0), cos(alpha),     sin(alpha));
    expected_A_r_Bz = Vector(PrimType_(0.0), -sin(alpha),    cos(alpha));
    checkGeometricalInterpretation(rotationBToA, expected_A_r_Bx, expected_A_r_By, expected_A_r_Bz);
  }

  inline static void checkGeometricalInterpretation(kindr::RotationQuaternion<PrimType_> rotationBToA,
                                                   Eigen::Matrix<PrimType_, 3, 1> expected_A_r_Bx,
                                                   Eigen::Matrix<PrimType_, 3, 1> expected_A_r_By,
                                                   Eigen::Matrix<PrimType_, 3, 1> expected_A_r_Bz) {

    typedef Eigen::Matrix<PrimType_, 3, 1> Vector;
    using std::cos;
    using std::sin;

    /* Consider two coordinate frames A and B. A is fixed, while B is rotated by alpha = pi/4 w.r.t. A.
     * The unit vectors of B in B frame are the vectors of the canonical base of R^3.
     * Using a geometrical approach, it can easily be shown that the components of these vectors expressed in A frame are:
     *
     *   A_r_Bx = [ cos(alpha) sin(alpha)  0]^T;
     *   A_r_By = [-sin(alpha) cos(alpha)  0]^T;
     *   A_r_Bz = [ 0          0           1]^T;
     *
     * Stacking these transformations column-wise returns a rotation matrix that projects the coordinates of a vector
     * expressed in the B frame to those of a vector expressed in A frame:
     *
     *  C_AB = [ A_r_Bx   A_r_By   A_r_Bz]
     */
    const Vector B_r_Bx = Vector::UnitX();
    const Vector B_r_By = Vector::UnitY();
    const Vector B_r_Bz = Vector::UnitZ();

    OtherRotation_ otherRotationBToA;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::convertToOtherRotation(otherRotationBToA, rotationBToA);

    Eigen::Matrix<PrimType_, 3, 3> eigenRotationMatrix;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::getRotationMatrixFromRotation(eigenRotationMatrix, otherRotationBToA);


    // Compute unit vectors with matrix multiplication
    Vector computed_A_r_Bx = eigenRotationMatrix*B_r_Bx;
    Vector computed_A_r_By = eigenRotationMatrix*B_r_By;
    Vector computed_A_r_Bz = eigenRotationMatrix*B_r_Bz;

    // Test unit vectors
    KINDR_ASSERT_DOUBLE_MX_EQ(expected_A_r_Bx, computed_A_r_Bx, 1.0e-3, "matrix multiply A_r_Bx");
    KINDR_ASSERT_DOUBLE_MX_EQ(expected_A_r_By, computed_A_r_By, 1.0e-3, "matrix multiply A_r_By");
    KINDR_ASSERT_DOUBLE_MX_EQ(expected_A_r_Bz, computed_A_r_Bz, 1.0e-3, "matrix multiply A_r_Bz");


    computed_A_r_Bx.setZero();
    computed_A_r_By.setZero();
    computed_A_r_Bz.setZero();


    // Compute unit vectors with custom rotation operator
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::rotateVector(computed_A_r_Bx, otherRotationBToA, B_r_Bx);
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::rotateVector(computed_A_r_By, otherRotationBToA, B_r_By);
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::rotateVector(computed_A_r_Bz, otherRotationBToA, B_r_Bz);

    // Test unit vectors
    KINDR_ASSERT_DOUBLE_MX_EQ(expected_A_r_Bx, computed_A_r_Bx, 1.0e-3, "rotate A_r_Bx");
    KINDR_ASSERT_DOUBLE_MX_EQ(expected_A_r_By, computed_A_r_By, 1.0e-3, "rotate A_r_By");
    KINDR_ASSERT_DOUBLE_MX_EQ(expected_A_r_Bz, computed_A_r_Bz, 1.0e-3, "rotate A_r_Bz");

  }

  inline static void testBoxPlus() {
    kindr::RotationQuaternion<PrimType_> kindrQuat(kindr::EulerAnglesZyx<PrimType_>(-0.4, 0.8, -0.5));
    Eigen::Matrix<PrimType_, 3, 1> kindrVelocity(0.5, 0.7, 0.9);
    OtherRotation_ otherQuat;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::convertToOtherRotation(otherQuat, kindrQuat);

    OtherVelocity_ otherVelocity;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::convertToOtherVelocityVector(otherVelocity, otherQuat, kindrVelocity);

    OtherRotation_ otherQuatOtherPlus;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::boxPlus(otherQuatOtherPlus, otherQuat, otherVelocity);
    kindr::RotationQuaternion<PrimType_> kindrQuatKindrPlus = kindrQuat.boxPlus(kindrVelocity);

    kindr::RotationQuaternion<PrimType_> kindrQuatOtherPlus;
    RotationConversion<OtherRotation_, OtherVelocity_, PrimType_>::convertToKindr(kindrQuatOtherPlus, otherQuatOtherPlus);

    // Test quaternions
    ASSERT_TRUE(kindrQuatKindrPlus.isNear(kindrQuatOtherPlus, 1.0e-3));

  }
};

} // namespace

