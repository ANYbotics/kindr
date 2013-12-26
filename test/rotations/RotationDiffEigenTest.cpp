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

template <typename ImplementationPair>
struct RotationDiffPairTest : public ::testing::Test {
  typedef typename ImplementationPair::first_type Rotation;
  typedef typename Rotation::Scalar RotationScalar;
  typedef typename ImplementationPair::second_type RotationDiff;
  typedef typename RotationDiff::Scalar RotationDiffScalar;
  typedef rot::LocalAngularVelocity<RotationDiffScalar, RotationDiff::Usage> LocalAngularVelocity;
  typedef typename rot::AngleAxis<RotationScalar, Rotation::Usage> AngleAxis;
  std::vector<Rotation> rotations;
  std::vector<LocalAngularVelocity> angularVelocities;

  RotationDiffPairTest() {
    rotations.push_back(Rotation(AngleAxis()));   // identity rotation
    rotations.push_back(Rotation(AngleAxis(kindr::common::NumTraits<RotationScalar>::dummy_precision()/10.0, 1, 0, 0)));  // small angle
    rotations.push_back(Rotation(AngleAxis(kindr::common::NumTraits<RotationScalar>::dummy_precision()/10.0, 0, 1, 0)));  // small angle
    rotations.push_back(Rotation(AngleAxis(kindr::common::NumTraits<RotationScalar>::dummy_precision()/10.0, 0, 0, 1)));  // small angle
    rotations.push_back(Rotation(AngleAxis(1.3, 1, 0, 0))); // large angle
    rotations.push_back(Rotation(AngleAxis(1.3, 0, 1, 0))); // large angle
    rotations.push_back(Rotation(AngleAxis(1.3, 0, 0, 1))); // large angle

    angularVelocities.push_back(LocalAngularVelocity());  // zero velocity
    angularVelocities.push_back(LocalAngularVelocity(kindr::common::NumTraits<RotationScalar>::dummy_precision()/10.0, 0.0, 0.0));
    angularVelocities.push_back(LocalAngularVelocity(0.0, kindr::common::NumTraits<RotationScalar>::dummy_precision()/10.0, 0.0));
    angularVelocities.push_back(LocalAngularVelocity(0.0, 0.0, kindr::common::NumTraits<RotationScalar>::dummy_precision()/10.0));
    angularVelocities.push_back(LocalAngularVelocity(0.1, 0.0, 0.0));
    angularVelocities.push_back(LocalAngularVelocity(0.0, 0.1, 0.0));
    angularVelocities.push_back(LocalAngularVelocity(0.0, 0.0, 0.1));

    angularVelocities.push_back(LocalAngularVelocity(2.2, 3.3, 4.4));
  }

};


typedef ::testing::Types<
  std::pair<rot::RotationQuaternionAD, rot::RotationQuaternionDiffAD>,
  std::pair<rot::RotationQuaternionAF, rot::RotationQuaternionDiffAF>,
  std::pair<rot::RotationQuaternionPD, rot::RotationQuaternionDiffPD>,
  std::pair<rot::RotationQuaternionPF, rot::RotationQuaternionDiffPF>,

  std::pair<rot::RotationVectorAD, rot::RotationVectorDiffAD>,
  std::pair<rot::RotationVectorAF, rot::RotationVectorDiffAF>,
  std::pair<rot::RotationVectorPD, rot::RotationVectorDiffPD>,
  std::pair<rot::RotationVectorPF, rot::RotationVectorDiffPF>,

  std::pair<rot::AngleAxisAD, rot::AngleAxisDiffAD>,
  std::pair<rot::AngleAxisAF, rot::AngleAxisDiffAF>,
  std::pair<rot::AngleAxisPD, rot::AngleAxisDiffPD>,
  std::pair<rot::AngleAxisPF, rot::AngleAxisDiffPF>,

  std::pair<rot::RotationMatrixAD, rot::RotationMatrixDiffAD>,
  std::pair<rot::RotationMatrixAF, rot::RotationMatrixDiffAF>,
  std::pair<rot::RotationMatrixPD, rot::RotationMatrixDiffPD>,
  std::pair<rot::RotationMatrixPF, rot::RotationMatrixDiffPF>,

  std::pair<rot::EulerAnglesZyxAD, rot::EulerAnglesZyxDiffAD>,
  std::pair<rot::EulerAnglesZyxAF, rot::EulerAnglesZyxDiffAF>,
  std::pair<rot::EulerAnglesZyxPD, rot::EulerAnglesZyxDiffPD>,
  std::pair<rot::EulerAnglesZyxPF, rot::EulerAnglesZyxDiffPF>,

  std::pair<rot::EulerAnglesXyzAD, rot::EulerAnglesXyzDiffAD>,
  std::pair<rot::EulerAnglesXyzAF, rot::EulerAnglesXyzDiffAF>,
  std::pair<rot::EulerAnglesXyzPD, rot::EulerAnglesXyzDiffPD>,
  std::pair<rot::EulerAnglesXyzPF, rot::EulerAnglesXyzDiffPF>
> TypeRotationAndRotationDiffPairs;

TYPED_TEST_CASE(RotationDiffPairTest, TypeRotationAndRotationDiffPairs);

TYPED_TEST(RotationDiffPairTest, testConversionToLocalAngularVelocity)
{
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;

  for (auto rotation : this->rotations) {
     for (auto angularVelocity : this->angularVelocities) {
       RotationDiff rotDiff(rotation, angularVelocity);
       LocalAngularVelocity angularVelocity2(rotation, rotDiff);
       ASSERT_NEAR(angularVelocity.x(),angularVelocity2.x(),1e-6) << "rotation: " << rotation << " angularVelocity: " << angularVelocity  << " angularVelocity2: " << angularVelocity2 << " rotDiff: " << rotDiff;
       ASSERT_NEAR(angularVelocity.y(),angularVelocity2.y(),1e-6) << "rotation: " << rotation << " angularVelocity: " << angularVelocity  << " angularVelocity2: " << angularVelocity2 << " rotDiff: " << rotDiff;
       ASSERT_NEAR(angularVelocity.z(),angularVelocity2.z(),1e-6) << "rotation: " << rotation << " angularVelocity: " << angularVelocity  << " angularVelocity2: " << angularVelocity2 << " rotDiff: " << rotDiff;
     }
  }

}

