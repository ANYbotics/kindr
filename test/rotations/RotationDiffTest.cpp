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

template <typename ImplementationPair>
struct RotationDiffPairTest : public ::testing::Test {
  typedef typename ImplementationPair::first_type Rotation;
  typedef typename Rotation::Scalar RotationScalar;
  typedef typename ImplementationPair::second_type RotationDiff;
  typedef typename RotationDiff::Scalar RotationDiffScalar;
  typedef rot::LocalAngularVelocity<RotationDiffScalar> LocalAngularVelocity;
  typedef typename rot::AngleAxis<RotationScalar> AngleAxis;
  std::vector<Rotation> rotations;
  std::vector<LocalAngularVelocity> localAngularVelocities;

  RotationDiffPairTest() {
    rotations.push_back(Rotation());   // identity rotation
    rotations.push_back(Rotation(AngleAxis(kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 1.0, 0, 0)));  // small angle
    rotations.push_back(Rotation(AngleAxis(kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 0, 1.0, 0)));  // small angle
    rotations.push_back(Rotation(AngleAxis(kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 0, 0, 1.0)));  // small angle
    rotations.push_back(Rotation(AngleAxis(1.3, 1.0, 0, 0))); // large angle
    rotations.push_back(Rotation(AngleAxis(1.3, 0, 1.0, 0))); // large angle
    rotations.push_back(Rotation(AngleAxis(1.3, 0, 0, 1.0))); // large angle

    localAngularVelocities.push_back(LocalAngularVelocity());  // zero velocity
    localAngularVelocities.push_back(LocalAngularVelocity(kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 0.0, 0.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.0, kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 0.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.0, 0.0, kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.1, 0.0, 0.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.0, 0.1, 0.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.0, 0.0, 0.1));
    localAngularVelocities.push_back(LocalAngularVelocity(2.2, 3.3, 4.4));
  }

};

typedef ::testing::Types<
  std::pair<rot::RotationQuaternionPD, rot::RotationQuaternionDiffPD>,
  std::pair<rot::RotationQuaternionPF, rot::RotationQuaternionDiffPF>,
  std::pair<rot::RotationMatrixPD, rot::RotationMatrixDiffPD>,
  std::pair<rot::RotationMatrixPF, rot::RotationMatrixDiffPF>,
  std::pair<rot::EulerAnglesZyxPD, rot::EulerAnglesZyxDiffPD>,
  std::pair<rot::EulerAnglesZyxPF, rot::EulerAnglesZyxDiffPF>,
  std::pair<rot::EulerAnglesXyzPD, rot::EulerAnglesXyzDiffPD>,
  std::pair<rot::EulerAnglesXyzPF, rot::EulerAnglesXyzDiffPF>
> TypeRotationAndRotationDiffPairs;

TYPED_TEST_CASE(RotationDiffPairTest, TypeRotationAndRotationDiffPairs);

TYPED_TEST(RotationDiffPairTest, testConversionToLocalAngularVelocity)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;

  for (auto rotation : this->rotations) {
     for (auto localAngularVelocity : this->localAngularVelocities) {
       RotationDiff rotDiff(rotation, localAngularVelocity);
       LocalAngularVelocity localAngularVelocity2(rotation, rotDiff);
//       LocalAngularVelocity localAngularVelocity3 = rotDiff.template cast<LocalAngularVelocity>(rotation);
       ASSERT_NEAR(localAngularVelocity.x(),localAngularVelocity2.x(),1e-3) << "rotation: " << rotation << " localAngularVelocity: " << localAngularVelocity  << " localAngularVelocity2: " << localAngularVelocity2 << " rotDiff: " << rotDiff;
       ASSERT_NEAR(localAngularVelocity.y(),localAngularVelocity2.y(),1e-3) << "rotation: " << rotation << " localAngularVelocity: " << localAngularVelocity  << " localAngularVelocity2: " << localAngularVelocity2 << " rotDiff: " << rotDiff;
       ASSERT_NEAR(localAngularVelocity.z(),localAngularVelocity2.z(),1e-3) << "rotation: " << rotation << " localAngularVelocity: " << localAngularVelocity  << " localAngularVelocity2: " << localAngularVelocity2 << " rotDiff: " << rotDiff;
     }
  }
}


template <typename ImplementationPair>
struct RotationDiffSingleTest : public ::testing::Test {
  typedef typename ImplementationPair::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationScalar;
  typedef typename ImplementationPair::second_type RotationQuaternionDiff;
  typedef typename RotationQuaternionDiff::Scalar RotationDiffScalar;
  typedef rot::LocalAngularVelocity<RotationDiffScalar> LocalAngularVelocity;
//  typedef rot::GlobalAngularVelocity<RotationDiffScalar, RotationDiff::Usage> GlobalAngularVelocity;
  typedef typename rot::AngleAxis<RotationScalar> AngleAxis;
  std::vector<RotationQuaternion> rotationQuaternions;
  std::vector<LocalAngularVelocity> localAngularVelocities;

  RotationDiffSingleTest() {
    rotationQuaternions.push_back(RotationQuaternion(AngleAxis()));   // identity rotation
    rotationQuaternions.push_back(RotationQuaternion(AngleAxis(kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 1.0, 0, 0)));  // small angle
    rotationQuaternions.push_back(RotationQuaternion(AngleAxis(kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 0, 1.0, 0)));  // small angle
    rotationQuaternions.push_back(RotationQuaternion(AngleAxis(kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 0, 0, 1.0)));  // small angle
    rotationQuaternions.push_back(RotationQuaternion(AngleAxis(1.3, 1.0, 0, 0))); // large angle
    rotationQuaternions.push_back(RotationQuaternion(AngleAxis(1.3, 0, 1.0, 0))); // large angle
    rotationQuaternions.push_back(RotationQuaternion(AngleAxis(1.3, 0, 0, 1.0))); // large angle

    localAngularVelocities.push_back(LocalAngularVelocity());  // zero velocity
    localAngularVelocities.push_back(LocalAngularVelocity(kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 0.0, 0.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.0, kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0, 0.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.0, 0.0, kindr::internal::NumTraits<RotationScalar>::dummy_precision()/10.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.1, 0.0, 0.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.0, 0.1, 0.0));
    localAngularVelocities.push_back(LocalAngularVelocity(0.0, 0.0, 0.1));
    localAngularVelocities.push_back(LocalAngularVelocity(2.2, 3.3, 4.4));
  }

};

typedef ::testing::Types<
  std::pair<rot::RotationQuaternionPD, rot::RotationQuaternionDiffPD>,
  std::pair<rot::RotationQuaternionPF, rot::RotationQuaternionDiffPF>
> QuaternionTypes;

TYPED_TEST_CASE(RotationDiffSingleTest, QuaternionTypes);

TYPED_TEST(RotationDiffSingleTest, testConversionToLocalAngularVelocity)
{
  typedef typename TestFixture::RotationQuaternionDiff RotationQuaternionDiff;
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename RotationQuaternion::Scalar Scalar;

  for (auto rotQuat : this->rotationQuaternions) {
     for (LocalAngularVelocity& localAngularVelocity : this->localAngularVelocities) {
       RotationQuaternionDiff rotQuatDiff(rotQuat, localAngularVelocity);

       LocalAngularVelocity localAngularVelocity2(2.0*rotQuat.getLocalQuaternionDiffMatrix()*rotQuatDiff.vector());
       ASSERT_NEAR(localAngularVelocity.x(),localAngularVelocity2.x(),1e-3) << "rquat: " << rotQuat;
       ASSERT_NEAR(localAngularVelocity.y(),localAngularVelocity2.y(),1e-3);
       ASSERT_NEAR(localAngularVelocity.z(),localAngularVelocity2.z(),1e-3);

       RotationQuaternionDiff rotQuatDiff2(0.5*rotQuat.getLocalQuaternionDiffMatrix().transpose()*localAngularVelocity.vector());
       ASSERT_NEAR(rotQuatDiff.w(),rotQuatDiff2.w(),1e-3);
       ASSERT_NEAR(rotQuatDiff.x(),rotQuatDiff2.x(),1e-3);
       ASSERT_NEAR(rotQuatDiff.y(),rotQuatDiff2.y(),1e-3);
       ASSERT_NEAR(rotQuatDiff.z(),rotQuatDiff2.z(),1e-3);

       // Finite Difference
       const Scalar dt = rot::internal::NumTraits<Scalar>::dummy_precision();
       RotationQuaternion rotQuatPert = rotQuat.boxPlus(rotQuat.rotate(localAngularVelocity.toImplementation())*dt);
       RotationQuaternionDiff rotQuatDiff3((rotQuatPert.w()-rotQuat.w())/dt,
                                           (rotQuatPert.x()-rotQuat.x())/dt,
                                           (rotQuatPert.y()-rotQuat.y())/dt,
                                           (rotQuatPert.z()-rotQuat.z())/dt);

       ASSERT_NEAR(rotQuatDiff.w(),rotQuatDiff3.w(),1e-2);
       ASSERT_NEAR(rotQuatDiff.x(),rotQuatDiff3.x(),1e-2);
       ASSERT_NEAR(rotQuatDiff.y(),rotQuatDiff3.y(),1e-2);
       ASSERT_NEAR(rotQuatDiff.z(),rotQuatDiff3.z(),1e-2);

     }
  }
}

TYPED_TEST(RotationDiffSingleTest, DISABLED_testConversionToGlobalAngularVelocity)
{
  typedef typename TestFixture::RotationQuaternionDiff RotationQuaternionDiff;
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename RotationQuaternion::Scalar Scalar;

  for (auto rotQuat : this->rotationQuaternions) { // qBI
     for (LocalAngularVelocity& localAngularVelocity : this->localAngularVelocities) { // IwIB
       RotationQuaternionDiff rotQuatDiff(0.5*rotQuat.getGlobalQuaternionDiffMatrix().transpose()*localAngularVelocity.vector());

       // Finite Difference
       const Scalar dt = rot::internal::NumTraits<Scalar>::dummy_precision();
       RotationQuaternion rotQuatPert = rotQuat.inverted().boxPlus(-localAngularVelocity.toImplementation()*dt).inverted(); // (qBI^-1 * exp(-IwIB*dt))^-1
       RotationQuaternionDiff rotQuatDiff3((rotQuatPert.w()-rotQuat.w())/dt,
                                           (rotQuatPert.x()-rotQuat.x())/dt,
                                           (rotQuatPert.y()-rotQuat.y())/dt,
                                           (rotQuatPert.z()-rotQuat.z())/dt);

       ASSERT_NEAR(rotQuatDiff.w(),rotQuatDiff3.w(),1e-2);
       ASSERT_NEAR(rotQuatDiff.x(),rotQuatDiff3.x(),1e-2);
       ASSERT_NEAR(rotQuatDiff.y(),rotQuatDiff3.y(),1e-2);
       ASSERT_NEAR(rotQuatDiff.z(),rotQuatDiff3.z(),1e-2);

     }
  }
}

