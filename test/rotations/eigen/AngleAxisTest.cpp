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


template <typename AngleAxisImplementation>
class AngleAxisSingleTest : public ::testing::Test{
 public:
  typedef AngleAxisImplementation AngleAxis;
  typedef typename AngleAxisImplementation::Scalar Scalar;
  typedef Eigen::Matrix<Scalar,4,1> Vector4;
  typedef Eigen::Matrix<Scalar,3,1> Vector;
  typedef typename rot::RotationQuaternion<typename AngleAxisImplementation::Scalar, AngleAxisImplementation::Usage> RotationQuaternion;
  Vector4 eigenVectorIdentity;
  Vector4 eigenVector4v1;
  Eigen::AngleAxis<Scalar> eigenAngleAxis;

//  const AngleAxis angleAxis1 = AngleAxis(vec1);
//  const AngleAxis angleAxis2 = AngleAxis(vec2);
  const AngleAxis angleAxisGeneric1 = AngleAxis(0.2, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
  const AngleAxis angleAxisGeneric1Plus2Pi = AngleAxis(0.2+2.0*M_PI, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
  const AngleAxis angleAxisGeneric1Minus2Pi = AngleAxis(0.2-2.0*M_PI, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
  const AngleAxis angleAxisGeneric2 = AngleAxis(0.6, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));

  const AngleAxis rotAngleAxisQuarterX = AngleAxis(M_PI/2.0, 1.0, 0.0, 0.0);
  const AngleAxis rotAngleAxisQuarterY = AngleAxis(M_PI/2.0, 0.0, 1.0, 0.0);
  const AngleAxis rotAngleAxisQuarterZ = AngleAxis(M_PI/2.0, 0.0, 0.0, 1.0);
  const AngleAxis rotAngleAxisIdentity = AngleAxis(0.0, 1.0, 0.0, 0.0);
  const Vector vec = Vector(1.3,-2.5,3.6);
  const Vector vecX = Vector(1.0,0.0,0.0);
  const Vector vecY = Vector(0.0,1.0,0.0);
  const Vector vecZ = Vector(0.0,0.0,1.0);

  AngleAxis rotAngleAxis1;
  AngleAxis rotAngleAxis2;

  AngleAxisSingleTest() {
    eigenVectorIdentity << 0.0, 1.0, 0.0, 0.0;
    eigenVector4v1 << 1.2, 2.0/sqrt(4+9+16), 3.0/sqrt(4+9+16), 4.0/sqrt(4+9+16);
    eigenAngleAxis.angle() = eigenVector4v1(0);
    eigenAngleAxis.axis() = eigenVector4v1.template block<3,1>(1,0);


    rotAngleAxis1 = AngleAxis(RotationQuaternion(0.0,0.36,0.48,0.8));
    rotAngleAxis2 = AngleAxis(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)));
  }
};

template <typename RotationQuaternionAngleAxisImplementationPair>
struct RotationQuaternionAngleAxisPairTest : public ::testing::Test{
  typedef typename RotationQuaternionAngleAxisImplementationPair::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationQuaternionAngleAxisImplementationPair::second_type AngleAxis;
  typedef typename AngleAxis::Scalar AngleAxisScalar;
  typedef Eigen::Matrix<RotationQuaternionScalar,3,1> Vector;

  const RotationQuaternionScalar rotQuatSmallNumber = kindr::common::NumTraits<RotationQuaternionScalar>::dummy_precision()/10.0;
  const RotationQuaternionScalar angleAxisSmallNumber = kindr::common::NumTraits<AngleAxisScalar>::dummy_precision()/10.0;

  const RotationQuaternion rotQuatQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuatQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuatQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(1.0,0.0,0.0,0.0);
  const RotationQuaternion rotQuat1 = RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
  const RotationQuaternion rotQuat1Conj = RotationQuaternion(4.0/sqrt(30.0),-3.0/sqrt(30.0),-1.0/sqrt(30.0),-2.0/sqrt(30.0));

  const AngleAxis rotAngleAxisQuarterX = AngleAxis(M_PI/2.0, 1.0, 0.0, 0.0);
  const AngleAxis rotAngleAxisQuarterY = AngleAxis(M_PI/2.0, 0.0, 1.0, 0.0);
  const AngleAxis rotAngleAxisQuarterZ = AngleAxis(M_PI/2.0, 0.0, 0.0, 1.0);
  const AngleAxis rotAngleAxisIdentity = AngleAxis(0.0, 1.0, 0.0, 0.0);


};


typedef ::testing::Types<
    rot::AngleAxisPD,
    rot::AngleAxisPF,
    rot::AngleAxisAD,
    rot::AngleAxisAF
> AngleAxisTypes;

typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPF, rot::AngleAxisPF>,
    std::pair<rot::RotationQuaternionPF, rot::AngleAxisPD>,
    std::pair<rot::RotationQuaternionPD, rot::AngleAxisPF>,
    std::pair<rot::RotationQuaternionPD, rot::AngleAxisPD>,
    std::pair<rot::RotationQuaternionAF, rot::AngleAxisAF>,
    std::pair<rot::RotationQuaternionAF, rot::AngleAxisAD>,
    std::pair<rot::RotationQuaternionAD, rot::AngleAxisAF>,
    std::pair<rot::RotationQuaternionAD, rot::AngleAxisAD>
> TypeQuaternionAngleAxisPairs;

TYPED_TEST_CASE(AngleAxisSingleTest, AngleAxisTypes);
TYPED_TEST_CASE(RotationQuaternionAngleAxisPairTest, TypeQuaternionAngleAxisPairs);




// --------------------------------------------------------------------------------------------------- //
// ------------------ Testing for constructors and getters for other rotation types ------------------ //
// --------------------------------------------------------------------------------------------------- //

// Testing constructors and getters for Rotation Vector
TYPED_TEST(AngleAxisSingleTest, testConstructors){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;

  AngleAxis rot;
  ASSERT_EQ(this->eigenVectorIdentity(0), rot.angle());
  ASSERT_EQ(this->eigenVectorIdentity(1), rot.axis().x());
  ASSERT_EQ(this->eigenVectorIdentity(2), rot.axis().y());
  ASSERT_EQ(this->eigenVectorIdentity(3), rot.axis().z());

  AngleAxis rot2(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rot2.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot2.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot2.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot2.axis().z());

  AngleAxis rot3(this->eigenVector4v1(0), this->eigenVector4v1.template block<3,1>(1,0));
  ASSERT_EQ(this->eigenVector4v1(0), rot3.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot3.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot3.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot3.axis().z());

  AngleAxis rot4(this->eigenVector4v1);
  ASSERT_EQ(this->eigenVector4v1(0), rot4.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot4.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot4.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot4.axis().z());


  AngleAxis rot5(this->eigenAngleAxis);
  ASSERT_EQ(this->eigenVector4v1(0), rot5.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot5.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot5.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot5.axis().z());


  AngleAxis rot6(rot4);
  ASSERT_EQ(this->eigenVector4v1(0), rot6.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot6.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot6.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot6.axis().z());

}

TYPED_TEST(AngleAxisSingleTest, testGetters)
{
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;

  AngleAxis rot(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rot.vector()(0));
  ASSERT_EQ(this->eigenVector4v1(1), rot.vector()(1));
  ASSERT_EQ(this->eigenVector4v1(2), rot.vector()(2));
  ASSERT_EQ(this->eigenVector4v1(3), rot.vector()(3));

  AngleAxis rot2(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rot2.toImplementation().angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot2.toImplementation().axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot2.toImplementation().axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot2.toImplementation().axis().z());

  AngleAxis rot3(this->angleAxisGeneric1Plus2Pi.getUnique());
  ASSERT_NEAR(this->angleAxisGeneric1.angle(), rot3.angle(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().x(), rot3.axis().x(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().y(), rot3.axis().y(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().z(), rot3.axis().z(),1e-6);

  AngleAxis rot4(this->angleAxisGeneric1Minus2Pi.getUnique());
  ASSERT_NEAR(this->angleAxisGeneric1.angle(), rot4.angle(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().x(), rot4.axis().x(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().y(), rot4.axis().y(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().z(), rot4.axis().z(),1e-6);

}

TYPED_TEST(AngleAxisSingleTest, testSetters)
{
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;

  AngleAxis rot(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  rot.setIdentity();
  ASSERT_EQ(this->eigenVectorIdentity(0), rot.angle());
  ASSERT_EQ(this->eigenVectorIdentity(1), rot.axis().x());
  ASSERT_EQ(this->eigenVectorIdentity(2), rot.axis().y());
  ASSERT_EQ(this->eigenVectorIdentity(3), rot.axis().z());

  AngleAxis rot2;
  rot2.setAngle(this->eigenVector4v1(0));
  rot2.setAxis(this->eigenVector4v1.template block<3,1>(1,0));
  ASSERT_EQ(this->eigenVector4v1(0), rot2.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot2.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot2.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot2.axis().z());

  AngleAxis rot3;
  rot3.setAngle(this->eigenVector4v1(0));
  rot3.setAxis(this->eigenVector4v1(1),this->eigenVector4v1(2),this->eigenVector4v1(3));
  ASSERT_EQ(this->eigenVector4v1(0), rot3.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot3.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot3.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot3.axis().z());

  AngleAxis rot4;
  rot4.setVector(this->eigenVector4v1);
  ASSERT_EQ(this->eigenVector4v1(0), rot4.angle());
  ASSERT_EQ(this->eigenVector4v1(1), rot4.axis().x());
  ASSERT_EQ(this->eigenVector4v1(2), rot4.axis().y());
  ASSERT_EQ(this->eigenVector4v1(3), rot4.axis().z());


  AngleAxis rot5(this->angleAxisGeneric1Plus2Pi);
  AngleAxis rot6 = rot5.setUnique();
  ASSERT_NEAR(this->angleAxisGeneric1.angle(), rot5.angle(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().x(), rot5.axis().x(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().y(), rot5.axis().y(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().z(), rot5.axis().z(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.angle(), rot6.angle(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().x(), rot6.axis().x(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().y(), rot6.axis().y(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().z(), rot6.axis().z(),1e-6);


  AngleAxis rot7(this->angleAxisGeneric1Plus2Pi);
  AngleAxis rot8 = rot7.setUnique();
  ASSERT_NEAR(this->angleAxisGeneric1.angle(), rot7.angle(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().x(), rot7.axis().x(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().y(), rot7.axis().y(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().z(), rot7.axis().z(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.angle(), rot8.angle(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().x(), rot8.axis().x(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().y(), rot8.axis().y(),1e-6);
  ASSERT_NEAR(this->angleAxisGeneric1.axis().z(), rot8.axis().z(),1e-6);

}



// Test Rotation Quaternion comparison (equality)
TYPED_TEST(AngleAxisSingleTest, testComparisonEqual){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  AngleAxis rot;

  // Check equality comparison
  rot = this->angleAxisGeneric1;
  ASSERT_EQ(true, rot==this->angleAxisGeneric1);
  ASSERT_EQ(true, rot==this->angleAxisGeneric1Plus2Pi);
  ASSERT_EQ(true, rot==this->angleAxisGeneric1Minus2Pi);
  ASSERT_EQ(false, rot==this->angleAxisGeneric2);
}

// Test Rotation Quaternion isNear
TYPED_TEST(AngleAxisSingleTest, testIsNear){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  AngleAxis rot;

  // Check isNear
  rot = this->angleAxisGeneric1;
  ASSERT_EQ(rot.isNear(this->angleAxisGeneric1,1e-6),true);
  ASSERT_EQ(rot.isNear(this->angleAxisGeneric1Plus2Pi,1e-6),true);
  ASSERT_EQ(rot.isNear(this->angleAxisGeneric1Minus2Pi,1e-6),true);
  ASSERT_EQ(rot.isNear(this->angleAxisGeneric2,1e-6),false);
}


// --------------------------------------------------------------------------------------------------- //
// -------- Testing for casting between different type of rotations and rotation Quaternions --------- //
// --------------------------------------------------------------------------------------------------- //


TYPED_TEST(RotationQuaternionAngleAxisPairTest, testConvertToRotationQuaternionUsingConstructor){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;

  RotationQuaternion rotQuat2(this->rotAngleAxisIdentity);
  ASSERT_NEAR(rotQuat2.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat2.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat2.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat2.z(), this->rotQuatIdentity.z(),1e-6);
  RotationQuaternion rotQuat3(this->rotAngleAxisQuarterX);
  ASSERT_NEAR(rotQuat3.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat3.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat3.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat3.z(), this->rotQuatQuarterX.z(),1e-6);
  RotationQuaternion rotQuat4(this->rotAngleAxisQuarterY);
  ASSERT_NEAR(rotQuat4.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat4.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat4.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat4.z(), this->rotQuatQuarterY.z(),1e-6);
  RotationQuaternion rotQuat5(this->rotAngleAxisQuarterZ);
  ASSERT_NEAR(rotQuat5.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat5.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat5.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat5.z(), this->rotQuatQuarterZ.z(),1e-6);
}

TYPED_TEST(RotationQuaternionAngleAxisPairTest, testConvertToAngleAxisUsingConstructor){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;

  AngleAxis angleAxis2(this->rotQuatIdentity);
  ASSERT_NEAR(angleAxis2.angle(), this->rotAngleAxisIdentity.angle(),1e-6);
  ASSERT_NEAR(angleAxis2.axis().x(), this->rotAngleAxisIdentity.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis2.axis().y(), this->rotAngleAxisIdentity.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis2.axis().z(), this->rotAngleAxisIdentity.axis().z(),1e-6);
  AngleAxis angleAxis3(this->rotQuatQuarterX);
  ASSERT_NEAR(angleAxis3.angle(), this->rotAngleAxisQuarterX.angle(),1e-6);
  ASSERT_NEAR(angleAxis3.axis().x(), this->rotAngleAxisQuarterX.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis3.axis().y(), this->rotAngleAxisQuarterX.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis3.axis().z(), this->rotAngleAxisQuarterX.axis().z(),1e-6);
  AngleAxis angleAxis4(this->rotQuatQuarterY);
  ASSERT_NEAR(angleAxis4.angle(), this->rotAngleAxisQuarterY.angle(),1e-6);
  ASSERT_NEAR(angleAxis4.axis().x(), this->rotAngleAxisQuarterY.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis4.axis().y(), this->rotAngleAxisQuarterY.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis4.axis().z(), this->rotAngleAxisQuarterY.axis().z(),1e-6);
  AngleAxis angleAxis5(this->rotQuatQuarterZ);
  ASSERT_NEAR(angleAxis5.angle(), this->rotAngleAxisQuarterY.angle(),1e-6);
  ASSERT_NEAR(angleAxis5.axis().x(), this->rotAngleAxisQuarterZ.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis5.axis().y(), this->rotAngleAxisQuarterZ.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis5.axis().z(), this->rotAngleAxisQuarterZ.axis().z(),1e-6);

}


TYPED_TEST(RotationQuaternionAngleAxisPairTest, testConvertToRotationQuaternionUsingAssignment) {
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;
  RotationQuaternion rotQuat;
  AngleAxis angleAxis;

  // TODO: add generic

  rotQuat = this->rotAngleAxisIdentity;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-6);
  rotQuat = this->rotAngleAxisQuarterX;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-6);
  rotQuat = this->rotAngleAxisQuarterY;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-6);
  rotQuat = this->rotAngleAxisQuarterZ;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-6);
}

TYPED_TEST(RotationQuaternionAngleAxisPairTest, testConvertToAngleAxisUsingAssignment) {
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;
  RotationQuaternion rotQuat;
  AngleAxis angleAxis;

  angleAxis = this->rotQuatIdentity;
  ASSERT_NEAR(angleAxis.angle(), this->rotAngleAxisIdentity.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->rotAngleAxisIdentity.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->rotAngleAxisIdentity.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->rotAngleAxisIdentity.axis().z(),1e-6);
  angleAxis = this->rotQuatQuarterX;
  ASSERT_NEAR(angleAxis.angle(), this->rotAngleAxisQuarterX.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->rotAngleAxisQuarterX.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->rotAngleAxisQuarterX.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->rotAngleAxisQuarterX.axis().z(),1e-6);
  angleAxis = this->rotQuatQuarterY;
  ASSERT_NEAR(angleAxis.angle(), this->rotAngleAxisQuarterY.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->rotAngleAxisQuarterY.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->rotAngleAxisQuarterY.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->rotAngleAxisQuarterY.axis().z(),1e-6);
  angleAxis = this->rotQuatQuarterZ;
  ASSERT_NEAR(angleAxis.angle(), this->rotAngleAxisQuarterY.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->rotAngleAxisQuarterZ.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->rotAngleAxisQuarterZ.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->rotAngleAxisQuarterZ.axis().z(),1e-6);
}

TYPED_TEST(RotationQuaternionAngleAxisPairTest, testConvertToRotationQuaternionUsingParenthesisOperator){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;
  RotationQuaternion rotQuat;
  AngleAxis angleAxis;


  rotQuat(this->rotAngleAxisIdentity);
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-6);
  rotQuat(this->rotAngleAxisQuarterX);
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-6);
  rotQuat(this->rotAngleAxisQuarterY);
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-6);
  rotQuat(this->rotAngleAxisQuarterZ);
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-6);
}

TYPED_TEST(RotationQuaternionAngleAxisPairTest, testConvertToAngleAxisUsingParenthesisOperator){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;
  RotationQuaternion rotQuat;
  AngleAxis angleAxis;

  angleAxis(this->rotQuatIdentity);
  ASSERT_NEAR(angleAxis.angle(), this->rotAngleAxisIdentity.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->rotAngleAxisIdentity.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->rotAngleAxisIdentity.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->rotAngleAxisIdentity.axis().z(),1e-6);
  angleAxis(this->rotQuatQuarterX);
  ASSERT_NEAR(angleAxis.angle(), this->rotAngleAxisQuarterX.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->rotAngleAxisQuarterX.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->rotAngleAxisQuarterX.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->rotAngleAxisQuarterX.axis().z(),1e-6);
  angleAxis(this->rotQuatQuarterY);
  ASSERT_NEAR(angleAxis.angle(), this->rotAngleAxisQuarterY.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->rotAngleAxisQuarterY.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->rotAngleAxisQuarterY.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->rotAngleAxisQuarterY.axis().z(),1e-6);
  angleAxis(this->rotQuatQuarterZ);
  ASSERT_NEAR(angleAxis.angle(), this->rotAngleAxisQuarterY.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->rotAngleAxisQuarterZ.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->rotAngleAxisQuarterZ.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->rotAngleAxisQuarterZ.axis().z(),1e-6);

}

// --------------------------------------------------------------------------------------------------- //
// ------------------------------------- Testing Angle Axis ------------------------------------------ //
// --------------------------------------------------------------------------------------------------- //

// Test Angle Axis Inversion
TYPED_TEST(RotationQuaternionAngleAxisPairTest, testAngleAxisInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;
  RotationQuaternion rotQuat;
  AngleAxis angleAxis1;
  AngleAxis angleAxis2;
  AngleAxis angleAxis3;
  AngleAxis angleAxis4;

  // Use rotation quaternion method as reference
  rotQuat = this->rotQuat1.inverted();
  angleAxis1 = rotQuat;

  // Use rotation vector method and compare
  angleAxis2 = this->rotQuat1;
  angleAxis2.invert();
  ASSERT_NEAR(angleAxis1.angle(),angleAxis2.angle(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().x(),angleAxis2.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().y(),angleAxis2.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().z(),angleAxis2.axis().z(),1e-6);

  angleAxis3 = this->rotQuat1;
  angleAxis4 = angleAxis3.inverted();
  ASSERT_NEAR(angleAxis1.angle(),angleAxis4.angle(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().x(),angleAxis4.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().y(),angleAxis4.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().z(),angleAxis4.axis().z(),1e-6);
}


// Test Angle Axis Vector Rotation
TYPED_TEST(AngleAxisSingleTest, testVectorRotation){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  AngleAxis rotQuat;
  Vector testVec;
  Vector testVec1;
  Vector testVec2;

  int signSwitch = 2*(AngleAxis::Usage == kindr::rotations::RotationUsage::ACTIVE)-1;

  // Check rotation of base vectors around main axis
  testVec = this->rotAngleAxisQuarterX.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotAngleAxisQuarterX.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotAngleAxisQuarterX.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),1e-6);
  testVec = this->rotAngleAxisQuarterY.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotAngleAxisQuarterY.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotAngleAxisQuarterY.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),1e-6);
  testVec = this->rotAngleAxisQuarterZ.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),1e-6);
  testVec = this->rotAngleAxisQuarterZ.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),1e-6);
  testVec = this->rotAngleAxisQuarterZ.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  testVec = this->rotAngleAxisQuarterX.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotAngleAxisQuarterX.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotAngleAxisQuarterX.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),1e-6);
  testVec = this->rotAngleAxisQuarterY.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotAngleAxisQuarterY.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotAngleAxisQuarterY.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),1e-6);
  testVec = this->rotAngleAxisQuarterZ.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),1e-6);
  testVec = this->rotAngleAxisQuarterZ.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),1e-6);
  testVec = this->rotAngleAxisQuarterZ.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  // Check rotation with Identity
  testVec = this->rotAngleAxisIdentity.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotAngleAxisIdentity.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotAngleAxisIdentity.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);
  testVec = this->rotAngleAxisIdentity.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotAngleAxisIdentity.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotAngleAxisIdentity.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  // Check combination between concatenation and rotate
  testVec1 = this->rotAngleAxis2.rotate(this->rotAngleAxis1.rotate(this->vec));
  testVec2 = (this->rotAngleAxis2*this->rotAngleAxis1).rotate(this->vec);
  ASSERT_NEAR(testVec1(0), testVec2(0),1e-6);
  ASSERT_NEAR(testVec1(1), testVec2(1),1e-6);
  ASSERT_NEAR(testVec1(2), testVec2(2),1e-6);
}


