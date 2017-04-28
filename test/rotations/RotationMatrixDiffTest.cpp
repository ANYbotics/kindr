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
struct RotationMatrixDiffTest: public ::testing::Test {
  typedef Implementation RotationDiff;
  typedef typename Implementation::Scalar Scalar;
  typedef rot::RotationMatrix<Scalar> Rotation;
  typedef rot::LocalAngularVelocity<Scalar> LocalAngularVelocity;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;


  Matrix3 eigenMatrix3vZero = Matrix3::Zero();
  Matrix3 eigenMatrix3v1;
  LocalAngularVelocity angularVelocity0 = LocalAngularVelocity(1.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity0y = LocalAngularVelocity(0.0, 1.0, 0.0);
  LocalAngularVelocity angularVelocity0z = LocalAngularVelocity(0.0, 0.0, 1.0);
  LocalAngularVelocity angularVelocity1 = LocalAngularVelocity(0.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity2 = LocalAngularVelocity(0.4, 0.3, 0.8)*0.1;
  LocalAngularVelocity angularVelocity3 = LocalAngularVelocity(40, 52, 99)*0.01;
  LocalAngularVelocity angularVelocity4 = LocalAngularVelocity(kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0, 0.0);
  LocalAngularVelocity angularVelocity5 = LocalAngularVelocity(0.0, kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0, 0.0);
  LocalAngularVelocity angularVelocity6 = LocalAngularVelocity(0.0, 0.0, kindr::internal::NumTraits<Scalar>::dummy_precision()/10.0);
  Rotation rotation1 = Rotation();
  Rotation rotation2 = Rotation(1.0, 0.0,  0.0,
                                0.0, 0.0,  1.0,
                                0.0, -1.0, 0.0); // psi=0, theta=0, phi=pi/2
  Rotation rotation3 = Rotation(0.0,   1.0, 0.0,
                                -1.0,  0.0, 0.0,
                                0.0 ,  0.0, 1.0); // psi=pi/2, theta=0, phi=0
  Rotation rotation4 = Rotation( 707.106781186548e-003,    0.00000000000000e+000,   -707.106781186547e-003,
                                 0.00000000000000e+000,    1.00000000000000e+000,    0.00000000000000e+000,
                                 707.106781186547e-003,    0.00000000000000e+000,    707.106781186548e-003); // psi=0, theta=pi/2, phi=0
  Rotation rotation5 = Rotation(   1.00000000000000e+000,   0.00000000000000e+000,    0.00000000000000e+000,
                                                               0.00000000000000e+000,    1.00000000000000e+000,    1.00000000000000e-015,
                                                               0.00000000000000e+000,   -1.00000000000000e-015,    1.00000000000000e+000); // psi=0, theta=0, phi=1e-15
  Rotation rotation6 = Rotation(    1.00000000000000e+000,    1.00000000000000e-015,    0.00000000000000e+000,
                                   -1.00000000000000e-015,    1.00000000000000e+000,    0.00000000000000e+000,
                                    0.00000000000000e+000,    0.00000000000000e+000,    1.00000000000000e+000); // psi=1e-15, theta=0, phi=0
  Rotation rotation7 = Rotation(1.00000000000000e+000,    0.00000000000000e+000,   -1.00000000000000e-015,
                                0.00000000000000e+000,    1.00000000000000e+000,    0.00000000000000e+000,
                                1.00000000000000e-015,    0.00000000000000e+000,    1.00000000000000e+000); // psi=0, theta=1e-15, phi=0
  Rotation rotation8 = Rotation(  879.923176281257e-003,    372.025551942260e-003,   -295.520206661340e-003,
                                  -327.579672728226e-003,    925.564159446682e-003,    189.796060978687e-003,
                                   344.131896020075e-003,   -70.1995402393384e-003,    936.293363584199e-003); //psi=0.4, theta=0.3 phi=0.2


  std::vector<Rotation> rotations;
  std::vector<LocalAngularVelocity> angularVelocities;
  std::vector<RotationDiff> rotDiffs;
  RotationMatrixDiffTest() {
    eigenMatrix3v1 << 1.1, 2.2, 3.3,
                                         4.4, 5.5, 6.6,
                                         7.7, 8.8, 9.9;
    rotations.push_back(rotation1);
    rotations.push_back(rotation2);
    rotations.push_back(rotation3);
    rotations.push_back(rotation4);
    rotations.push_back(rotation5);
    rotations.push_back(rotation6);
    rotations.push_back(rotation7);
    rotations.push_back(rotation8);
    angularVelocities.push_back(angularVelocity0);
    angularVelocities.push_back(angularVelocity0y);
    angularVelocities.push_back(angularVelocity0z);
    angularVelocities.push_back(angularVelocity1);
    angularVelocities.push_back(angularVelocity2);
    angularVelocities.push_back(angularVelocity3);
    angularVelocities.push_back(angularVelocity4);
    angularVelocities.push_back(angularVelocity5);
    angularVelocities.push_back(angularVelocity6);
    rotDiffs.push_back(RotationDiff((rotation1.matrix()+rotation2.matrix())*1e-3));
    rotDiffs.push_back(RotationDiff((rotation2.matrix()+rotation3.matrix())*1e-3));
    rotDiffs.push_back(RotationDiff((rotation4.matrix()+rotation5.matrix())*1e-3));
    rotDiffs.push_back(RotationDiff((rotation6.matrix()+rotation7.matrix())*1e-3));
  }
};


typedef ::testing::Types<
    rot::RotationMatrixDiffPD,
    rot::RotationMatrixDiffPF
> RotationMatrixDiffTypes;

TYPED_TEST_CASE(RotationMatrixDiffTest, RotationMatrixDiffTypes);


TYPED_TEST(RotationMatrixDiffTest, testConstructors)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;

  RotationDiff rotDiff;
  ASSERT_EQ(this->eigenMatrix3vZero(0,0), rotDiff.matrix()(0,0));
  ASSERT_EQ(this->eigenMatrix3vZero(0,1), rotDiff.matrix()(0,1));
  ASSERT_EQ(this->eigenMatrix3vZero(0,2), rotDiff.matrix()(0,2));
  ASSERT_EQ(this->eigenMatrix3vZero(1,0), rotDiff.matrix()(1,0));
  ASSERT_EQ(this->eigenMatrix3vZero(1,1), rotDiff.matrix()(1,1));
  ASSERT_EQ(this->eigenMatrix3vZero(1,2), rotDiff.matrix()(2,2));
  ASSERT_EQ(this->eigenMatrix3vZero(2,0), rotDiff.matrix()(2,0));
  ASSERT_EQ(this->eigenMatrix3vZero(2,1), rotDiff.matrix()(2,1));
  ASSERT_EQ(this->eigenMatrix3vZero(2,2), rotDiff.matrix()(2,2));


  RotationDiff rotDiff2(this->eigenMatrix3v1(0,0), this->eigenMatrix3v1(0,1), this->eigenMatrix3v1(0,2),
                        this->eigenMatrix3v1(1,0), this->eigenMatrix3v1(1,1), this->eigenMatrix3v1(1,2),
                        this->eigenMatrix3v1(2,0), this->eigenMatrix3v1(2,1), this->eigenMatrix3v1(2,2)
                        );
  ASSERT_EQ(this->eigenMatrix3v1(0,0), rotDiff2.matrix()(0,0));
  ASSERT_EQ(this->eigenMatrix3v1(0,1), rotDiff2.matrix()(0,1));
  ASSERT_EQ(this->eigenMatrix3v1(0,2), rotDiff2.matrix()(0,2));
  ASSERT_EQ(this->eigenMatrix3v1(1,0), rotDiff2.matrix()(1,0));
  ASSERT_EQ(this->eigenMatrix3v1(1,1), rotDiff2.matrix()(1,1));
  ASSERT_EQ(this->eigenMatrix3v1(1,2), rotDiff2.matrix()(1,2));
  ASSERT_EQ(this->eigenMatrix3v1(2,0), rotDiff2.matrix()(2,0));
  ASSERT_EQ(this->eigenMatrix3v1(2,1), rotDiff2.matrix()(2,1));
  ASSERT_EQ(this->eigenMatrix3v1(2,2), rotDiff2.matrix()(2,2));

  RotationDiff rotDiff3(this->eigenMatrix3v1);
  ASSERT_EQ(this->eigenMatrix3v1(0,0), rotDiff3.matrix()(0,0));
  ASSERT_EQ(this->eigenMatrix3v1(0,1), rotDiff3.matrix()(0,1));
  ASSERT_EQ(this->eigenMatrix3v1(0,2), rotDiff3.matrix()(0,2));
  ASSERT_EQ(this->eigenMatrix3v1(1,0), rotDiff3.matrix()(1,0));
  ASSERT_EQ(this->eigenMatrix3v1(1,1), rotDiff3.matrix()(1,1));
  ASSERT_EQ(this->eigenMatrix3v1(1,2), rotDiff3.matrix()(1,2));
  ASSERT_EQ(this->eigenMatrix3v1(2,0), rotDiff3.matrix()(2,0));
  ASSERT_EQ(this->eigenMatrix3v1(2,1), rotDiff3.matrix()(2,1));
  ASSERT_EQ(this->eigenMatrix3v1(2,2), rotDiff3.matrix()(2,2));

}


TYPED_TEST(RotationMatrixDiffTest, testSetters)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;


  RotationDiff rotDiff(this->eigenMatrix3v1);
  rotDiff.setZero();
  ASSERT_EQ(this->eigenMatrix3vZero(0,0), rotDiff.matrix()(0,0));
  ASSERT_EQ(this->eigenMatrix3vZero(0,1), rotDiff.matrix()(0,1));
  ASSERT_EQ(this->eigenMatrix3vZero(0,2), rotDiff.matrix()(0,2));
  ASSERT_EQ(this->eigenMatrix3vZero(1,0), rotDiff.matrix()(1,0));
  ASSERT_EQ(this->eigenMatrix3vZero(1,1), rotDiff.matrix()(1,1));
  ASSERT_EQ(this->eigenMatrix3vZero(1,2), rotDiff.matrix()(2,2));
  ASSERT_EQ(this->eigenMatrix3vZero(2,0), rotDiff.matrix()(2,0));
  ASSERT_EQ(this->eigenMatrix3vZero(2,1), rotDiff.matrix()(2,1));
  ASSERT_EQ(this->eigenMatrix3vZero(2,2), rotDiff.matrix()(2,2));

  RotationDiff rotDiff2;
  rotDiff2.matrix() = this->eigenMatrix3v1;
  ASSERT_EQ(this->eigenMatrix3v1(0,0), rotDiff2.matrix()(0,0));
  ASSERT_EQ(this->eigenMatrix3v1(0,1), rotDiff2.matrix()(0,1));
  ASSERT_EQ(this->eigenMatrix3v1(0,2), rotDiff2.matrix()(0,2));
  ASSERT_EQ(this->eigenMatrix3v1(1,0), rotDiff2.matrix()(1,0));
  ASSERT_EQ(this->eigenMatrix3v1(1,1), rotDiff2.matrix()(1,1));
  ASSERT_EQ(this->eigenMatrix3v1(1,2), rotDiff2.matrix()(1,2));
  ASSERT_EQ(this->eigenMatrix3v1(2,0), rotDiff2.matrix()(2,0));
  ASSERT_EQ(this->eigenMatrix3v1(2,1), rotDiff2.matrix()(2,1));
  ASSERT_EQ(this->eigenMatrix3v1(2,2), rotDiff2.matrix()(2,2));

}


TYPED_TEST(RotationMatrixDiffTest, testFiniteDifference)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::RotationDiff::Matrix3x3 Matrix3;

 const  double dt = 1e-5;
  for (auto& rotation : this->rotations) {
    for (auto& angularVelocity : this->angularVelocities) {
      // Finite difference method for checking derivatives
      RotationDiff rotationDiff(rotation, angularVelocity);
      Rotation rotationNext = rotation.boxPlus(dt*rotation.rotate(angularVelocity.vector()));
      Matrix3 dmat = (rotationNext.toImplementation()-rotation.toImplementation())/dt;
      ASSERT_NEAR(rotationDiff.toImplementation()(0,0),dmat(0,0),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(0,1),dmat(0,1),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(0,2),dmat(0,2),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(1,0),dmat(1,0),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(1,1),dmat(1,1),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(1,2),dmat(1,2),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(2,0),dmat(2,0),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(2,1),dmat(2,1),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(2,2),dmat(2,2),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
    }
  }
}

TYPED_TEST(RotationMatrixDiffTest, testFiniteDifferenceInverse)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::RotationDiff::Matrix3x3 Matrix3;
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;

 const  double dt = 1e-5;
  for (auto& rotation : this->rotations) {
    for (auto& rotDiff : this->rotDiffs) {
      // Finite difference method for checking derivatives
      RotationDiff rotationDiff(rotDiff);
      LocalAngularVelocity angularVelocity(rotation, rotDiff);
      Rotation rotationNext = rotation.boxPlus(dt*rotation.rotate(angularVelocity.vector()));
      Matrix3 dmat = (rotationNext.toImplementation()-rotation.toImplementation())/dt;
      ASSERT_NEAR(rotationDiff.toImplementation()(0,0),dmat(0,0),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(0,1),dmat(0,1),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(0,2),dmat(0,2),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(1,0),dmat(1,0),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(1,1),dmat(1,1),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(1,2),dmat(1,2),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(2,0),dmat(2,0),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(2,1),dmat(2,1),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
      ASSERT_NEAR(rotationDiff.toImplementation()(2,2),dmat(2,2),1e-2) << "angularVelocity: " << angularVelocity << " \nrotation: \n" << rotation << " \nrotationNext: \n" << rotationNext <<" \ndiff: \n" << rotationDiff << " \ndmat: \n" << dmat;
    }
  }
}


