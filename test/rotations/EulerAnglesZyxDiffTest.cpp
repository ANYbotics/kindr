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


template <typename Implementation>
struct EulerAnglesZyxDiffTest: public ::testing::Test {
  typedef Implementation RotationDiff;
  typedef typename Implementation::Scalar Scalar;
  typedef rot::EulerAnglesZyx<Scalar> Rotation;
  typedef rot::LocalAngularVelocity<Scalar> LocalAngularVelocity;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;


  Vector3 eigenVector3vZero = Vector3::Zero();
  Vector3 eigenVector3v1 = Vector3(2.2, 3.3, 4.4);

  LocalAngularVelocity angularVelocity1 = LocalAngularVelocity(0.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity2 = LocalAngularVelocity(0.4, 0.3, 0.8)*0.1;
  LocalAngularVelocity angularVelocity4 = LocalAngularVelocity(kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity5 = LocalAngularVelocity(0.0, kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0);
  LocalAngularVelocity angularVelocity6 = LocalAngularVelocity(0.0, 0.0, kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0);
  Rotation rotation1 = Rotation(0.1, 0.1, 0.1);
  Rotation rotation2 = Rotation(0.3, 0.0, 0.0);
  Rotation rotation3 = Rotation(0.0, 0.3, 0.0);
  Rotation rotation4 = Rotation(0.0, 0.0, 0.3);
  Rotation rotation5 = Rotation(kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 0.0);
  Rotation rotation6 = Rotation(0.0, kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0);
  Rotation rotation7 = Rotation(0.0, 0.0, kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0);
  Rotation rotation8 = Rotation(0.8, 0.9, 0.6);

  std::vector<Rotation> rotations;
  std::vector<LocalAngularVelocity> angularVelocities;
  EulerAnglesZyxDiffTest() {
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
    angularVelocities.push_back(angularVelocity4);
    angularVelocities.push_back(angularVelocity5);
    angularVelocities.push_back(angularVelocity6);

  }
};


typedef ::testing::Types<
    rot::EulerAnglesZyxDiffPD,
    rot::EulerAnglesZyxDiffPF
> EulerAnglesZyxDiffTypes;

TYPED_TEST_CASE(EulerAnglesZyxDiffTest, EulerAnglesZyxDiffTypes);


TYPED_TEST(EulerAnglesZyxDiffTest, testConstructors)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;

  RotationDiff rotDiff;
  ASSERT_EQ(this->eigenVector3vZero(0), rotDiff.z());
  ASSERT_EQ(this->eigenVector3vZero(1), rotDiff.y());
  ASSERT_EQ(this->eigenVector3vZero(2), rotDiff.x());

  RotationDiff rotDiff2(this->eigenVector3v1(0), this->eigenVector3v1(1), this->eigenVector3v1(2));
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff2.z());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff2.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff2.x());


  RotationDiff rotDiff4(this->eigenVector3v1);
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff4.z());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff4.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff4.x());

}

TYPED_TEST(EulerAnglesZyxDiffTest, testGetters)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;


  RotationDiff rotDiff(this->eigenVector3v1(0), this->eigenVector3v1(1), this->eigenVector3v1(2));
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff.toImplementation()(0));
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff.toImplementation()(1));
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff.toImplementation()(2));


  RotationDiff rotDiff2(this->eigenVector3v1(0), this->eigenVector3v1(1), this->eigenVector3v1(2));
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff2.yaw());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff2.pitch());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff2.roll());


}

TYPED_TEST(EulerAnglesZyxDiffTest, testSetters)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;


  RotationDiff rotDiff;
  rotDiff.z() = this->eigenVector3v1(0);
  rotDiff.y() = this->eigenVector3v1(1);
  rotDiff.x() = this->eigenVector3v1(2);
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff.z());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff.x());

  RotationDiff rotDiff2;
  rotDiff2.toImplementation() = this->eigenVector3v1;
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff2.z());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff2.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff2.x());

  RotationDiff rotDiff3(this->eigenVector3v1(0), this->eigenVector3v1(1), this->eigenVector3v1(2));
  rotDiff3.setZero();
  ASSERT_EQ(this->eigenVector3vZero(0), rotDiff3.z());
  ASSERT_EQ(this->eigenVector3vZero(1), rotDiff3.y());
  ASSERT_EQ(this->eigenVector3vZero(2), rotDiff3.x());

  RotationDiff rotDiff4;
  rotDiff4.yaw() = this->eigenVector3v1(0);
  rotDiff4.pitch() = this->eigenVector3v1(1);
  rotDiff4.roll() = this->eigenVector3v1(2);
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff4.z());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff4.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff4.x());

}


TYPED_TEST(EulerAnglesZyxDiffTest, testFiniteDifference)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef  typename TestFixture::Vector3 Vector3;

  const  double dt = 1.0e-3;
  for (auto rotation : this->rotations) {
    for (auto angularVelocity2 : this->angularVelocities) {
      RotationDiff rotationDiff(angularVelocity2.toImplementation());
      rot::LocalAngularVelocity<Scalar> angularVelocity(rotation, rotationDiff);
      // Finite difference method for checking derivatives

      Rotation rotationNext = rotation.boxPlus(dt*rotation.rotate(angularVelocity.vector()));
      rotationNext.setUnique();
      rotation.setUnique();
      Vector3 dn = (rotationNext.toImplementation()-rotation.toImplementation())/dt;

      ASSERT_NEAR(rotationDiff.z(),dn(0),1e-3) << " angular velocity: " << angularVelocity << " rotation: " << rotation << " rotationNext: " << rotationNext  << " diff: " << rotationDiff  << " approxdiff: " << dn.transpose();
      ASSERT_NEAR(rotationDiff.y(),dn(1),1e-3)  << " angular velocity: " << angularVelocity  <<  "rotation: " << rotation << " rotationNext: " << rotationNext << " diff: " << rotationDiff << " approxdiff: " << dn.transpose();
      ASSERT_NEAR(rotationDiff.x(),dn(2),1e-3) << " angular velocity: " << angularVelocity << " rotation: " << rotation << " rotationNext: " << rotationNext  << " diff: " << rotationDiff << " approxdiff: " << dn.transpose();

    }
  }
}

TYPED_TEST(EulerAnglesZyxDiffTest, testFiniteDifferenceInverse)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef  typename TestFixture::Vector3 Vector3;

  const  double dt = 1.0e-3;
  for (auto rotation : this->rotations) {
    for (auto angularVelocity : this->angularVelocities) {
      // Finite difference method for checking derivatives
      RotationDiff rotationDiff(rotation, angularVelocity);
      Rotation rotationNext = rotation.boxPlus(dt*rotation.rotate(angularVelocity.vector()));
      rotationNext.setUnique();
      rotation.setUnique();
      Vector3 dn = (rotationNext.toImplementation()-rotation.toImplementation())/dt;

      ASSERT_NEAR(rotationDiff.z(),dn(0),1e-3) << " angular velocity: " << angularVelocity << " rotation: " << rotation << " rotationNext: " << rotationNext  << " diff: " << rotationDiff  << " approxdiff: " << dn.transpose();
      ASSERT_NEAR(rotationDiff.y(),dn(1),1e-3)  << " angular velocity: " << angularVelocity  <<  "rotation: " << rotation << " rotationNext: " << rotationNext << " diff: " << rotationDiff << " approxdiff: " << dn.transpose();
      ASSERT_NEAR(rotationDiff.x(),dn(2),1e-3) << " angular velocity: " << angularVelocity << " rotation: " << rotation << " rotationNext: " << rotationNext  << " diff: " << rotationDiff << " approxdiff: " << dn.transpose();

    }
  }
}

TYPED_TEST(EulerAnglesZyxDiffTest, mappingLocalAngularVelocity)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef  typename TestFixture::Vector3 Vector3;

  for (auto rotation : this->rotations) {
    for (auto angularVelocity : this->angularVelocities) {
      RotationDiff rotationDiff1(rotation, angularVelocity);
      RotationDiff rotationDiff2(rotation.getMappingFromLocalAngularVelocityToDiff()*angularVelocity.vector());
      ASSERT_NEAR(rotationDiff1.x(),rotationDiff2.x(),1e-3) << " angular velocity: " << angularVelocity << " rotation: " << rotation   << " diff1: " << rotationDiff1  << " diff2: " << rotationDiff2;
      ASSERT_NEAR(rotationDiff1.y(),rotationDiff2.y(),1e-3)  << " angular velocity: " << angularVelocity  <<  "rotation: " << rotation  << " diff1: " << rotationDiff1 << " diff2: " << rotationDiff2;
      ASSERT_NEAR(rotationDiff1.z(),rotationDiff2.z(),1e-3) << " angular velocity: " << angularVelocity << " rotation: " << rotation   << " diff1: " << rotationDiff1 << " diff2: " << rotationDiff2;
    }
  }
}

TYPED_TEST(EulerAnglesZyxDiffTest, mappingLocalAngularVelocityInverse)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef  typename TestFixture::Vector3 Vector3;
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;

  for (auto rotation : this->rotations) {
    for (auto angularVelocity : this->angularVelocities) {
      RotationDiff rotationDiff1(rotation, angularVelocity);
      LocalAngularVelocity angularVelocity2(rotation.getMappingFromDiffToLocalAngularVelocity()*rotationDiff1.vector());
      ASSERT_NEAR(angularVelocity.x(),angularVelocity2.x(),1e-3) << " angular velocity: " << angularVelocity << " rotation: " << rotation << " angularVelocity1: " << angularVelocity  << " angularVelocity2: " << angularVelocity2;
      ASSERT_NEAR(angularVelocity.y(),angularVelocity2.y(),1e-3)  << " angular velocity: " << angularVelocity  <<  "rotation: " << rotation << " angularVelocity1: " << angularVelocity << " angularVelocity2: " << angularVelocity2;
      ASSERT_NEAR(angularVelocity.z(),angularVelocity2.z(),1e-3) << " angular velocity: " << angularVelocity << " rotation: " << rotation << " angularVelocity1: " << angularVelocity << " angularVelocity2: " << angularVelocity2;
    }
  }
}
