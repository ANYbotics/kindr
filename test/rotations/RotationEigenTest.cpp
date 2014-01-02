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

#include "kindr/common/gtest_eigen.hpp"
#include "kindr/quaternions/QuaternionEigen.hpp"
#include "kindr/rotations/RotationEigen.hpp"

namespace rot = kindr::rotations::eigen_impl;
namespace quat = kindr::quaternions::eigen_impl;



template <typename ImplementationPair_>
struct ConversionTest : public ::testing::Test{
  typedef typename ImplementationPair_::first_type RotationA;
  typedef typename ImplementationPair_::second_type RotationB;

  RotationA rotA;
  RotationB rotB;

};

template <typename Rotation_>
struct RotationQuaternionTestType {
  typedef Rotation_ RotationQuaternion;
  typedef typename RotationQuaternion::Scalar Scalar;

  const RotationQuaternion rotQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const RotationQuaternion rotIdentity = RotationQuaternion(1.0,0.0,0.0,0.0);

  RotationQuaternion rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-6) {
    ASSERT_NEAR(rotA.w(), rotB.w(), tol);
    ASSERT_NEAR(rotA.x(), rotB.x(), tol);
    ASSERT_NEAR(rotA.y(), rotB.y(), tol);
    ASSERT_NEAR(rotA.z(), rotB.z(), tol);
  }
};


template <typename Rotation_>
struct RotationVectorTestType {
  typedef Rotation_ RotationVector;
  typedef typename RotationVector::Scalar Scalar;

  const RotationVector rotQuarterX = RotationVector(M_PI/2.0,0.0,0.0);
  const RotationVector rotQuarterY = RotationVector(0.0,M_PI/2.0,0.0);
  const RotationVector rotQuarterZ = RotationVector(0.0,0.0,M_PI/2.0);
  const RotationVector rotIdentity = RotationVector(0.0,0.0,0.0);

  RotationVector rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-6) {
    ASSERT_NEAR(rotA.x(), rotB.x(), tol);
    ASSERT_NEAR(rotA.y(), rotB.y(), tol);
    ASSERT_NEAR(rotA.z(), rotB.z(), tol);
  }
};

template <typename Rotation_>
struct AngleAxisTestType {
  typedef Rotation_ Rotation;
  typedef typename Rotation::Scalar Scalar;

  const Rotation rotQuarterX = Rotation(M_PI/2.0, 1.0, 0.0, 0.0);
  const Rotation rotQuarterY = Rotation(M_PI/2.0, 0.0, 1.0, 0.0);
  const Rotation rotQuarterZ = Rotation(M_PI/2.0, 0.0, 0.0, 1.0);
  const Rotation rotIdentity = Rotation(0.0, 1.0, 0.0, 0.0);

  Rotation rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-6) {
    ASSERT_NEAR(rotA.angle(), rotB.angle(), tol);
    ASSERT_NEAR(rotA.axis().x(), rotB.axis().x(), tol);
    ASSERT_NEAR(rotA.axis().y(), rotB.axis().y(), tol);
    ASSERT_NEAR(rotA.axis().z(), rotB.axis().z(), tol);
  }
};


template <typename Rotation_>
struct RotationMatrixTestType {
  typedef Rotation_ Rotation;
  typedef typename Rotation::Scalar Scalar;

  Rotation rotQuarterX;
  Rotation rotQuarterY;
  Rotation rotQuarterZ;
  Rotation rotIdentity;

  Rotation rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-4) {
    KINDR_ASSERT_DOUBLE_MX_EQ(rotA.toStoredImplementation(), rotB.toStoredImplementation(), tol, "matrix");
  }

  RotationMatrixTestType() {
    if (Rotation_::Usage == kindr::rotations::RotationUsage::PASSIVE) {
      rotQuarterX = Rotation( 1.0,  0.0,  0.0,
                              0.0,  0.0,  1.0,
                              0.0, -1.0,  0.0); // psi=0, theta=0, phi=pi/2

      rotQuarterY = Rotation( 0.0,  0.0,  -1.0,
                            0.0,  1.0,  0.0,
                            1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

      rotQuarterZ = Rotation( 0.0,  1.0,  0.0,
                              -1.0,  0.0,  0.0,
                               0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0

    }
    else {
      rotQuarterX = Rotation( 1.0,  0.0,  0.0,
                              0.0,  0.0,  -1.0,
                              0.0, 1.0,  0.0); // psi=0, theta=0, phi=pi/2

      rotQuarterY = Rotation( 0.0,  0.0,  1.0,
                            0.0,  1.0,  0.0,
                           -1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

      rotQuarterZ = Rotation( 0.0,  -1.0,  0.0,
                              1.0,  0.0,  0.0,
                              0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0
     }

    rotIdentity = Rotation( 1.0,  0.0,  0.0,
                            0.0,  1.0,  0.0,
                            0.0,  0.0,  1.0);
  }
};


typedef ::testing::Types<
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationVectorTestType<rot::RotationVectorPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationVectorTestType<rot::RotationVectorPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationVectorTestType<rot::RotationVectorPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationVectorTestType<rot::RotationVectorPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAF>, RotationVectorTestType<rot::RotationVectorAF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAF>, RotationVectorTestType<rot::RotationVectorAD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAD>, RotationVectorTestType<rot::RotationVectorAF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAD>, RotationVectorTestType<rot::RotationVectorAD>>,

    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationMatrixTestType<rot::RotationMatrixPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationMatrixTestType<rot::RotationMatrixPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAF>, RotationMatrixTestType<rot::RotationMatrixAF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAF>, RotationMatrixTestType<rot::RotationMatrixAD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAD>, RotationMatrixTestType<rot::RotationMatrixAF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAD>, RotationMatrixTestType<rot::RotationMatrixAD>>,

    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAF>, AngleAxisTestType<rot::AngleAxisAF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAF>, AngleAxisTestType<rot::AngleAxisAD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAD>, AngleAxisTestType<rot::AngleAxisAF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionAD>, AngleAxisTestType<rot::AngleAxisAD>>,

    std::pair<RotationVectorTestType<rot::RotationVectorPF>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, RotationMatrixTestType<rot::RotationMatrixPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, RotationMatrixTestType<rot::RotationMatrixPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorAF>, RotationMatrixTestType<rot::RotationMatrixAF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorAF>, RotationMatrixTestType<rot::RotationMatrixAD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorAD>, RotationMatrixTestType<rot::RotationMatrixAF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorAD>, RotationMatrixTestType<rot::RotationMatrixAD>>,

    std::pair<RotationVectorTestType<rot::RotationVectorPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorAF>, AngleAxisTestType<rot::AngleAxisAF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorAF>, AngleAxisTestType<rot::AngleAxisAD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorAD>, AngleAxisTestType<rot::AngleAxisAF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorAD>, AngleAxisTestType<rot::AngleAxisAD>>,


    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixAF>, AngleAxisTestType<rot::AngleAxisAF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixAF>, AngleAxisTestType<rot::AngleAxisAD>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixAD>, AngleAxisTestType<rot::AngleAxisAF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixAD>, AngleAxisTestType<rot::AngleAxisAD>>

> TypeRotationPairs;


TYPED_TEST_CASE(ConversionTest, TypeRotationPairs);

TYPED_TEST(ConversionTest, testConversions) {

  this->rotA.rot = this->rotB.rotIdentity;
  this->rotA.assertNear(this->rotA.rotIdentity, this->rotA.rot);

  this->rotA.rot = this->rotB.rotQuarterX;
  this->rotA.assertNear(this->rotA.rotQuarterX, this->rotA.rot);

  this->rotA.rot = this->rotB.rotQuarterY;
  this->rotA.assertNear(this->rotA.rotQuarterY, this->rotA.rot);

  this->rotA.rot = this->rotB.rotQuarterZ;
  this->rotA.assertNear(this->rotA.rotQuarterZ, this->rotA.rot);

  /* vice versa */
  this->rotB.rot = this->rotA.rotIdentity;
  this->rotB.assertNear(this->rotB.rotIdentity, this->rotB.rot);

  this->rotB.rot = this->rotA.rotQuarterX;
  this->rotB.assertNear(this->rotB.rotQuarterX, this->rotB.rot);

  this->rotB.rot = this->rotA.rotQuarterY;
  this->rotB.assertNear(this->rotB.rotQuarterY, this->rotB.rot);

  this->rotB.rot = this->rotA.rotQuarterZ;
  this->rotB.assertNear(this->rotB.rotQuarterZ, this->rotB.rot);

}














