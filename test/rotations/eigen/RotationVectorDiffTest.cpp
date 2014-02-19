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

#include <tuple>
namespace rot = kindr::rotations::eigen_impl;


template <typename Implementation>
struct RotationVectorDiffTest: public ::testing::Test {
  typedef Implementation RotationDiff;
  typedef typename Implementation::Scalar Scalar;
  typedef rot::RotationVector<Scalar, RotationDiff::Usage> Rotation;
  typedef rot::LocalAngularVelocity<Scalar, RotationDiff::Usage> LocalAngularVelocity;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;


  Vector3 eigenVector3vZero = Vector3::Zero();
  Vector3 eigenVector3v1 = Vector3(2.2, 3.3, 4.4);

  LocalAngularVelocity angularVelocity1 = LocalAngularVelocity(0.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity2 = LocalAngularVelocity(kindr::common::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity3 = LocalAngularVelocity(0.0, kindr::common::NumTraits<Scalar>::dummy_precision()/10.0, 0.0);
  LocalAngularVelocity angularVelocity4 = LocalAngularVelocity(0.0, 0.0, kindr::common::NumTraits<Scalar>::dummy_precision()/10.0);
  LocalAngularVelocity angularVelocity5 = LocalAngularVelocity(0.1, 0.3, 0.2);
  LocalAngularVelocity angularVelocity6 = LocalAngularVelocity(0.4, 0.3, 0.8);

  Rotation rotation1 = Rotation(0.0, 0.0, 0.0);
  Rotation rotation2 = Rotation(kindr::common::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 0.0);
  Rotation rotation3 = Rotation(0.0, kindr::common::NumTraits<Scalar>::dummy_precision()/10.0, 0.0);
  Rotation rotation4 = Rotation(0.0, 0.0, kindr::common::NumTraits<Scalar>::dummy_precision()/10.0);
  Rotation rotation5 = Rotation(0.1, 0.1, 0.1);
  Rotation rotation6 = Rotation(0.1, 1.3, 0.1);
  Rotation rotation7 = Rotation(0.1, 0.1, 1.3);
  Rotation rotation8 = Rotation(0.8, 0.9, 1.2);

  std::vector<Rotation> rotations;
  std::vector<LocalAngularVelocity> angularVelocities;
  std::vector<double> dts;
  std::vector<std::tuple<Rotation, LocalAngularVelocity, double>> values;
  RotationVectorDiffTest() {
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

    dts.push_back(1e-3);
    dts.push_back(1e-4);
    dts.push_back(1e-4);
    dts.push_back(1e-3);
    dts.push_back(1e-3);
    dts.push_back(1e-3);

    values.push_back(std::make_tuple(rotation1, angularVelocity1, 1e-3));
    values.push_back(std::make_tuple(rotation2, angularVelocity1, 1e-3));
    values.push_back(std::make_tuple(rotation3, angularVelocity1, 1e-3));
    values.push_back(std::make_tuple(rotation4, angularVelocity1, 1e-3));
    values.push_back(std::make_tuple(rotation5, angularVelocity1, 1e-3));
    values.push_back(std::make_tuple(rotation6, angularVelocity1, 1e-3));
    values.push_back(std::make_tuple(rotation7, angularVelocity1, 1e-3));
    values.push_back(std::make_tuple(rotation8, angularVelocity1, 1e-3));

    values.push_back(std::make_tuple(rotation1, angularVelocity2, 1e-3));
    values.push_back(std::make_tuple(rotation2, angularVelocity2, 1e-3));
    values.push_back(std::make_tuple(rotation3, angularVelocity2, 1e-3));
    values.push_back(std::make_tuple(rotation4, angularVelocity2, 1e-3));
    values.push_back(std::make_tuple(rotation5, angularVelocity2, 1e-3));
    values.push_back(std::make_tuple(rotation6, angularVelocity2, 1e-3));
    values.push_back(std::make_tuple(rotation7, angularVelocity2, 1e-3));
    values.push_back(std::make_tuple(rotation8, angularVelocity2, 1e-3));

    values.push_back(std::make_tuple(rotation1, angularVelocity3, 1e-3));
    values.push_back(std::make_tuple(rotation2, angularVelocity3, 1e-3));
    values.push_back(std::make_tuple(rotation3, angularVelocity3, 1e-3));
    values.push_back(std::make_tuple(rotation4, angularVelocity3, 1e-3));
    values.push_back(std::make_tuple(rotation5, angularVelocity3, 1e-3));
    values.push_back(std::make_tuple(rotation6, angularVelocity3, 1e-3));
    values.push_back(std::make_tuple(rotation7, angularVelocity3, 1e-3));
    values.push_back(std::make_tuple(rotation8, angularVelocity3, 1e-3));


    values.push_back(std::make_tuple(rotation1, angularVelocity4, 1e-3));
    values.push_back(std::make_tuple(rotation2, angularVelocity4, 1e-3));
    values.push_back(std::make_tuple(rotation3, angularVelocity4, 1e-3));
    values.push_back(std::make_tuple(rotation4, angularVelocity4, 1e-3));
    values.push_back(std::make_tuple(rotation5, angularVelocity4, 1e-3));
    values.push_back(std::make_tuple(rotation6, angularVelocity4, 1e-3));
    values.push_back(std::make_tuple(rotation7, angularVelocity4, 1e-3));
    values.push_back(std::make_tuple(rotation8, angularVelocity4, 1e-3));

    values.push_back(std::make_tuple(rotation1, angularVelocity5, 1e-3));
    values.push_back(std::make_tuple(rotation2, angularVelocity5, 1e-3));
    values.push_back(std::make_tuple(rotation3, angularVelocity5, 1e-3));
    values.push_back(std::make_tuple(rotation4, angularVelocity5, 1e-3));
    values.push_back(std::make_tuple(rotation5, angularVelocity5, 1e-5));
    values.push_back(std::make_tuple(rotation6, angularVelocity5, 1e-6));
    values.push_back(std::make_tuple(rotation7, angularVelocity5, 1e-5));
    values.push_back(std::make_tuple(rotation8, angularVelocity5, 1e-5));

//    values.push_back(std::make_tuple(rotation1, angularVelocity6, 1e-3));
//    values.push_back(std::make_tuple(rotation2, angularVelocity6, 1e-3));
//    values.push_back(std::make_tuple(rotation3, angularVelocity6, 1e-3));
//    values.push_back(std::make_tuple(rotation4, angularVelocity6, 1e-3));
//    values.push_back(std::make_tuple(rotation5, angularVelocity6, 1e-3));
//    values.push_back(std::make_tuple(rotation6, angularVelocity6, 1e-3));
//    values.push_back(std::make_tuple(rotation7, angularVelocity6, 1e-3));
//    values.push_back(std::make_tuple(rotation8, angularVelocity6, 1e-3));

  }
};


typedef ::testing::Types<
    rot::RotationVectorDiffAD,
    rot::RotationVectorDiffAF,
    rot::RotationVectorDiffPD,
    rot::RotationVectorDiffPF
> RotationVectorDiffTypes;

TYPED_TEST_CASE(RotationVectorDiffTest, RotationVectorDiffTypes);


TYPED_TEST(RotationVectorDiffTest, testConstructors)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;

  RotationDiff rotDiff;
  ASSERT_EQ(this->eigenVector3vZero(0), rotDiff.x());
  ASSERT_EQ(this->eigenVector3vZero(1), rotDiff.y());
  ASSERT_EQ(this->eigenVector3vZero(2), rotDiff.z());

  RotationDiff rotDiff2(this->eigenVector3v1(0), this->eigenVector3v1(1), this->eigenVector3v1(2));
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff2.x());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff2.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff2.z());


  RotationDiff rotDiff4(this->eigenVector3v1);
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff4.x());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff4.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff4.z());

}

TYPED_TEST(RotationVectorDiffTest, testGetters)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;


  RotationDiff rotDiff(this->eigenVector3v1(0), this->eigenVector3v1(1), this->eigenVector3v1(2));
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff.vector().x());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff.vector().y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff.vector().z());

}

TYPED_TEST(RotationVectorDiffTest, testSetters)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;


  RotationDiff rotDiff;
  rotDiff.x() = this->eigenVector3v1(0);
  rotDiff.y() = this->eigenVector3v1(1);
  rotDiff.z() = this->eigenVector3v1(2);
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff.x());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff.z());

  RotationDiff rotDiff2;
  rotDiff2.vector() = this->eigenVector3v1;
  ASSERT_EQ(this->eigenVector3v1(0), rotDiff2.x());
  ASSERT_EQ(this->eigenVector3v1(1), rotDiff2.y());
  ASSERT_EQ(this->eigenVector3v1(2), rotDiff2.z());

  RotationDiff rotDiff3(this->eigenVector3v1(0), this->eigenVector3v1(1), this->eigenVector3v1(2));
  rotDiff3.setZero();
  ASSERT_EQ(this->eigenVector3vZero(0), rotDiff3.x());
  ASSERT_EQ(this->eigenVector3vZero(1), rotDiff3.y());
  ASSERT_EQ(this->eigenVector3vZero(2), rotDiff3.z());

}


TYPED_TEST(RotationVectorDiffTest, testFiniteDifference)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::RotationDiff::Vector3 Vector3;


  for (auto value : this->values) {


    auto rotation = std::get<0>(value);
    auto angularVelocity = std::get<1>(value);
    auto dt = std::get<2>(value);
     // Finite difference method for checking derivatives
     RotationDiff rotationDiff(rotation, angularVelocity);
     Rotation rotationNext = rotation.boxPlus(dt*angularVelocity.toImplementation());
     const Vector3 diff = rotationNext.vector()-rotation.vector();
     Vector3 dn = diff/dt;


     ASSERT_NEAR(rotationDiff.x(),dn(0),1e-1) << "angular velocity: " << angularVelocity << "\nrotation: " << rotation << "\nrotationNext: " << rotationNext  << "\ndiff: " << rotationDiff  << " \napproxdiff: " << dn.transpose();
     ASSERT_NEAR(rotationDiff.y(),dn(1),1e-1)  << "angular velocity: " << angularVelocity  <<  "\nrotation: " << rotation << "\nrotationNext: " << rotationNext << "\ndiff: " << rotationDiff << " \napproxdiff: " << dn.transpose();
     ASSERT_NEAR(rotationDiff.z(),dn(2),1e-1) << "angular velocity: " << angularVelocity << "\nrotation: " << rotation << "\nrotationNext: " << rotationNext  << "\ndiff: " << rotationDiff << " \napproxdiff: " << dn.transpose();


  }

}

