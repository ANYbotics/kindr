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

#include "kindr/rotations/RotationDiff.hpp"
#include "kindr/common/gtest_eigen.hpp"
#include "kindr/common/common.hpp"

namespace rot = kindr;
namespace quat = kindr;

template <typename Implementation>
struct RotationQuaternionDiffTest: public ::testing::Test {
  typedef Implementation RotationDiff;
  typedef typename Implementation::Scalar Scalar;
  typedef rot::RotationQuaternion<Scalar> Rotation;
  typedef rot::LocalAngularVelocity<Scalar> LocalAngularVelocity;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
  typedef quat::Quaternion<Scalar> Quaternion;
  Vector4 eigenVector4vZero = Vector4::Zero();
  Vector4 eigenVector4v1;

  LocalAngularVelocity angularVelocity1 = LocalAngularVelocity(0.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity2 = LocalAngularVelocity(0.4, 0.3, 0.8);
  LocalAngularVelocity angularVelocity3 = LocalAngularVelocity(40, 52, 99)*0.001;
  LocalAngularVelocity angularVelocity4 = LocalAngularVelocity(kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity5 = LocalAngularVelocity(0.0, kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0);
  LocalAngularVelocity angularVelocity6 = LocalAngularVelocity(0.0, 0.0, kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0);
  Rotation rotation1 = Rotation(Quaternion(0.0, 1.0, 0.0, 0.0).toUnitQuaternion());
  Rotation rotation2 = Rotation(Quaternion(0.5, 1.0, 0.0, 0.0).toUnitQuaternion());
  Rotation rotation3 = Rotation(Quaternion(0.5, 0.0, 1.0, 0.0).toUnitQuaternion());
  Rotation rotation4 = Rotation(Quaternion(0.5, 0.0, 0.0, 1.0).toUnitQuaternion());
  Rotation rotation5 = Rotation(Quaternion(kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 1.0, 0.0, 0.0).toUnitQuaternion());
  Rotation rotation6 = Rotation(Quaternion(kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 1.0, 0.0).toUnitQuaternion());
  Rotation rotation7 = Rotation(Quaternion(kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 0.0, 1.0).toUnitQuaternion());
  Rotation rotation8 = Rotation(Quaternion(0.8, 1.0/sqrt(1+4+9), 2.0/sqrt(1+4+9), 3.0/sqrt(1+4+9)).toUnitQuaternion());

  std::vector<Rotation> rotations;
  std::vector<LocalAngularVelocity> angularVelocities;
  RotationQuaternionDiffTest() {
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
    rot::RotationQuaternionDiffPD,
    rot::RotationQuaternionDiffPF
> RotationQuaternionDiffTypes;

TYPED_TEST_CASE(RotationQuaternionDiffTest, RotationQuaternionDiffTypes);


TYPED_TEST(RotationQuaternionDiffTest, testConstructors)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Quaternion Quaternion;

  RotationDiff rotDiff;
  ASSERT_EQ(this->eigenVector4vZero(0), rotDiff.w());
  ASSERT_EQ(this->eigenVector4vZero(1), rotDiff.x());
  ASSERT_EQ(this->eigenVector4vZero(2), rotDiff.y());
  ASSERT_EQ(this->eigenVector4vZero(3), rotDiff.z());

  RotationDiff rotDiff2(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff2.w());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff2.x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff2.y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff2.z());

  RotationDiff rotDiff3(this->eigenVector4v1(0), this->eigenVector4v1.template block<3,1>(1,0));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff3.w());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff3.x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff3.y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff3.z());

  RotationDiff rotDiff4(this->eigenVector4v1);
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff4.w());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff4.x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff4.y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff4.z());

  RotationDiff rotDiff5(Quaternion(this->eigenVector4v1));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff5.w());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff5.x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff5.y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff5.z());
}


TYPED_TEST(RotationQuaternionDiffTest, testGetters)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;

  RotationDiff rotDiff2(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff2.vector()(0));
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff2.vector()(1));
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff2.vector()(2));
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff2.vector()(3));


  RotationDiff rotDiff3(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff3.toQuaternion().w());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff3.toQuaternion().x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff3.toQuaternion().y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff3.toQuaternion().z());

  RotationDiff rotDiff4(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff3.real());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff3.imaginary().x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff3.imaginary().y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff3.imaginary().z());

}

TYPED_TEST(RotationQuaternionDiffTest, testSetters)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationDiff rotDiff3;
  rotDiff3.toQuaternion() = Quaternion(this->eigenVector4v1);
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff3.w());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff3.x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff3.y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff3.z());

  RotationDiff rotDiff4(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  rotDiff4.setZero();
  ASSERT_EQ(this->eigenVector4vZero(0), rotDiff4.w());
  ASSERT_EQ(this->eigenVector4vZero(1), rotDiff4.x());
  ASSERT_EQ(this->eigenVector4vZero(2), rotDiff4.y());
  ASSERT_EQ(this->eigenVector4vZero(3), rotDiff4.z());


  RotationDiff rotDiff5;
  rotDiff5.w() = this->eigenVector4v1(0);
  rotDiff5.x() = this->eigenVector4v1(1);
  rotDiff5.y() = this->eigenVector4v1(2);
  rotDiff5.z() = this->eigenVector4v1(3);
  ASSERT_EQ(this->eigenVector4v1(0), rotDiff5.w());
  ASSERT_EQ(this->eigenVector4v1(1), rotDiff5.x());
  ASSERT_EQ(this->eigenVector4v1(2), rotDiff5.y());
  ASSERT_EQ(this->eigenVector4v1(3), rotDiff5.z());

}


TYPED_TEST(RotationQuaternionDiffTest, testFiniteDifference)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::RotationDiff::Imaginary Vector3;

  const  double dt = 1e-5;
  for (auto rotation : this->rotations) {
    for (auto angularVelocity : this->angularVelocities) {
      // Finite difference method for checking derivatives
      RotationDiff rotationDiff(rotation, angularVelocity);
      Rotation rotationNext = rotation.boxPlus(dt*rotation.rotate(angularVelocity.vector()));
      Scalar dreal = (rotationNext.real()-rotation.real())/dt;
      Vector3 dimag = (rotationNext.imaginary()-rotation.imaginary())/dt;
      ASSERT_NEAR(rotationDiff.w(),dreal,1e-2) << "rotation: " << rotation << " angular velocity: " << angularVelocity <<     " rdiff: " << dreal << " " << dimag(0) << " " << dimag(1) << " " << dimag(2) << "diff: " << rotationDiff;
      ASSERT_NEAR(rotationDiff.x(),dimag(0),1e-2) << "rotation: " << rotation << " angular velocity: " << angularVelocity  << " rdiff: " << dreal << " " << dimag(0) << " " << dimag(1) << " " << dimag(2) << " diff: " << rotationDiff;
      ASSERT_NEAR(rotationDiff.y(),dimag(1),1e-2) << "rotation: " << rotation << " angular velocity: " << angularVelocity <<  " rdiff: " << dreal << " " << dimag(0) << " " << dimag(1) << " " << dimag(2) << " diff: " << rotationDiff;
      ASSERT_NEAR(rotationDiff.z(),dimag(2),1e-2) << "rotation: " << rotation << " angular velocity: " << angularVelocity <<  " rdiff: " << dreal << " " << dimag(0) << " " << dimag(1) << " " << dimag(2) << " diff: " << rotationDiff;

    }
  }
}
