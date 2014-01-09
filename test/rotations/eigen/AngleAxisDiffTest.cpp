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


#include <iostream>

#include <Eigen/Core>

#include <gtest/gtest.h>

#include "kindr/rotations/RotationDiffEigen.hpp"
#include "kindr/common/gtest_eigen.hpp"
#include "kindr/common/common.hpp"

namespace rot = kindr::rotations::eigen_impl;



template <typename Implementation>
struct AngleAxisDiffTest: public ::testing::Test {
  typedef Implementation RotationDiff;

  typedef typename Implementation::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

//  template<enum kindr::rotations::RotationUsage Usage_ = Implementation::Usage>
  typedef rot::AngleAxis<Scalar, RotationDiff::Usage> Rotation;
//  template<enum kindr::rotations::RotationUsage Usage_ = Implementation::Usage>
  typedef rot::LocalAngularVelocity<Scalar, RotationDiff::Usage> LocalAngularVelocity;


  Vector4 eigenVector4vZero = Vector4::Zero();
  Vector4 eigenVector4v1;

  LocalAngularVelocity angularVelocity1 = LocalAngularVelocity(0.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity2 = LocalAngularVelocity(0.4, 0.3, 0.8);
  LocalAngularVelocity angularVelocity3 = LocalAngularVelocity(40, 52, 99);
  LocalAngularVelocity angularVelocity4 = LocalAngularVelocity(kindr::common::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity5 = LocalAngularVelocity(0.0, kindr::common::NumTraits<Scalar>::dummy_precision()/10.0, 0.0);
  LocalAngularVelocity angularVelocity6 = LocalAngularVelocity(0.0, 0.0, kindr::common::NumTraits<Scalar>::dummy_precision()/10.0);

  Rotation rotation1 = Rotation(0.0, 1.0, 0.0, 0.0);
  Rotation rotation2 = Rotation(0.5, 1.0, 0.0, 0.0);
  Rotation rotation3 = Rotation(0.5, 0.0, 1.0, 0.0);
  Rotation rotation4 = Rotation(0.5, 0.0, 0.0, 1.0);
  Rotation rotation5 = Rotation(1e-6, 1.0, 0.0, 0.0);
  Rotation rotation6 = Rotation(1e-6, 0.0, 1.0, 0.0);
  Rotation rotation7 = Rotation(1e-6, 0.0, 0.0, 1.0);
  Rotation rotation8 = Rotation(0.8, 1.0/sqrt(1+4+9), 2.0/sqrt(1+4+9), 3.0/sqrt(1+4+9));

  std::vector<Rotation> rotations;
  std::vector<LocalAngularVelocity> angularVelocities;
  AngleAxisDiffTest() {
    rotations.push_back(rotation1);
    rotations.push_back(rotation2);
    rotations.push_back(rotation3);
    rotations.push_back(rotation4);
    rotations.push_back(rotation5);
    rotations.push_back(rotation6);
    rotations.push_back(rotation7);
    rotations.push_back(rotation8);

    angularVelocities.push_back(angularVelocity1);
    angularVelocities.push_back(angularVelocity2);
    angularVelocities.push_back(angularVelocity3);
    angularVelocities.push_back(angularVelocity4);
    angularVelocities.push_back(angularVelocity5);
    angularVelocities.push_back(angularVelocity6);

    eigenVector4v1 << 2.2, 3.3, 4.4, 5.5;
  }
};



typedef ::testing::Types<
    rot::AngleAxisDiffAD,
    rot::AngleAxisDiffAF,
    rot::AngleAxisDiffPD,
    rot::AngleAxisDiffPF
> AngleAxisDiffTypes;

TYPED_TEST_CASE(AngleAxisDiffTest, AngleAxisDiffTypes);



TYPED_TEST(AngleAxisDiffTest, testAngleAxisDiffConstructors)
{
  typedef typename TestFixture::RotationDiff AngleAxisDiff;
  typedef typename TestFixture::Scalar Scalar;

  AngleAxisDiff rotDiff;
  ASSERT_EQ(this->eigenVector4vZero(0), rotDiff.angle());
  ASSERT_EQ(this->eigenVector4vZero(1), rotDiff.axis().x());
  ASSERT_EQ(this->eigenVector4vZero(2), rotDiff.axis().y());
  ASSERT_EQ(this->eigenVector4vZero(3), rotDiff.axis().z());

  AngleAxisDiff rotDiff2(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff2.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff2.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff2.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff2.axis().z());

  AngleAxisDiff rotDiff3(this->eigenVector4v1(0), this->eigenVector4v1.template block<3,1>(1,0));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff3.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff3.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff3.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff3.axis().z());

  AngleAxisDiff rotDiff4(this->eigenVector4v1);
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff4.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff4.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff4.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff4.axis().z());
}


TYPED_TEST(AngleAxisDiffTest, testAngleAxisDiffGetters)
{
  typedef typename TestFixture::RotationDiff AngleAxisDiff;
  typedef typename TestFixture::Scalar Scalar;

  AngleAxisDiff rotDiff(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff.vector()(0));
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff.vector()(1));
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff.vector()(2));
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff.vector()(3));
}

TYPED_TEST(AngleAxisDiffTest, testAngleAxisDiffSetters)
{
  typedef typename TestFixture::RotationDiff AngleAxisDiff;
  typedef typename TestFixture::Scalar Scalar;

  AngleAxisDiff rotDiff(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  rotDiff.setZero();
  ASSERT_EQ(this->eigenVector4vZero(0), rotDiff.angle());
  ASSERT_EQ(this->eigenVector4vZero(1), rotDiff.axis().x());
  ASSERT_EQ(this->eigenVector4vZero(2), rotDiff.axis().y());
  ASSERT_EQ(this->eigenVector4vZero(3), rotDiff.axis().z());

  AngleAxisDiff rotDiff2;
  rotDiff2.angle() = this->eigenVector4v1(0);
  rotDiff2.axis()(0) = this->eigenVector4v1(1);
  rotDiff2.axis()(1) = this->eigenVector4v1(2);
  rotDiff2.axis()(2) = this->eigenVector4v1(3);
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff2.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff2.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff2.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff2.axis().z());
}


TYPED_TEST(AngleAxisDiffTest, testFiniteDifference)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::RotationDiff::Vector3 Vector3;

 const  double dt = 0.00000001;
  for (auto rotation : this->rotations) {
    for (auto angularVelocity : this->angularVelocities) {
      // Finite difference method for checking derivatives
      RotationDiff rotationDiff(rotation, angularVelocity);
      Rotation rotationNext = rotation.boxPlus(dt*angularVelocity.toImplementation());
      Scalar dtheta = (rotationNext.angle()-rotation.angle())/dt;
      Vector3 dn = (rotationNext.axis()-rotation.axis())/dt;
      ASSERT_NEAR(rotationDiff.angle(),dtheta,1e-6) << "rotation: " << rotation << " angular velocity: " << angularVelocity << " angleAxisDiff: " << rotationDiff;
      ASSERT_NEAR(rotationDiff.axis()(0),dn(0),1e-6) << "rotation: " << rotation << " angular velocity: " << angularVelocity  << " angleAxisDiff: " << rotationDiff;
      ASSERT_NEAR(rotationDiff.axis()(1),dn(1),1e-6) << "rotation: " << rotation << " angular velocity: " << angularVelocity << " angleAxisDiff: " << rotationDiff;
      ASSERT_NEAR(rotationDiff.axis()(2),dn(2),1e-6) << "rotation: " << rotation << " angular velocity: " << angularVelocity << " angleAxisDiff: " << rotationDiff;

    }
  }
}
