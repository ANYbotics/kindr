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
  typedef Eigen::Matrix<Scalar,4,1> Vector;

  Vector eigenVectorIdentity;
  Vector eigenVector4v1;
  Eigen::AngleAxis<Scalar> eigenAngleAxis;

//  const AngleAxis angleAxis1 = AngleAxis(vec1);
//  const AngleAxis angleAxis2 = AngleAxis(vec2);
  const AngleAxis angleAxisIdentity = AngleAxis();
  const AngleAxis angleAxisGeneric1 = AngleAxis(0.2, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
  const AngleAxis angleAxisGeneric2 = AngleAxis(0.2*2.0*M_PI, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
  AngleAxisSingleTest() {
    eigenVectorIdentity << 0.0, 1.0, 0.0, 0.0;
    eigenVector4v1 << 1.2, 2.0/sqrt(4+9+16), 3.0/sqrt(4+9+16), 4.0/sqrt(4+9+16);
    eigenAngleAxis.angle() = eigenVector4v1(0);
    eigenAngleAxis.axis() = eigenVector4v1.template block<3,1>(1,0);
  }
};

template <typename RotationQuaternionAngleAxisImplementationPair>
struct RotationQuaternionAngleAxisPairTest : public ::testing::Test{
  typedef typename RotationQuaternionAngleAxisImplementationPair::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationQuaternionAngleAxisImplementationPair::second_type AngleAxis;
  typedef typename AngleAxis::Scalar AngleAxisScalar;

  const RotationQuaternionScalar rotQuatSmallNumber = kindr::common::NumTraits<RotationQuaternionScalar>::dummy_precision()/10.0;
  const RotationQuaternionScalar angleAxisSmallNumber = kindr::common::NumTraits<AngleAxisScalar>::dummy_precision()/10.0;

  const RotationQuaternion rotQuatQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuatQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuatQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(1.0,0.0,0.0,0.0);
  const RotationQuaternion rotQuat1 = RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
  const RotationQuaternion rotQuat1Conj = RotationQuaternion(4.0/sqrt(30.0),-3.0/sqrt(30.0),-1.0/sqrt(30.0),-2.0/sqrt(30.0));

  const AngleAxis angleAxisQuarterX = AngleAxis(M_PI/2.0, 1.0, 0.0, 0.0);
  const AngleAxis angleAxisQuarterY = AngleAxis(M_PI/2.0, 0.0, 1.0, 0.0);
  const AngleAxis angleAxisQuarterZ = AngleAxis(M_PI/2.0, 0.0, 0.0, 1.0);
  const AngleAxis angleAxisIdentity = AngleAxis(0.0, 1.0, 0.0, 0.0);


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

}



// Test Rotation Quaternion comparison (equality)
TYPED_TEST(AngleAxisSingleTest, testComparisonEqual){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  AngleAxis rot;

  // Check equality comparison
  rot = this->angleAxisGeneric1;
  ASSERT_EQ(rot==this->angleAxisGeneric1,true);
  ASSERT_EQ(rot==this->angleAxisGeneric2,false);
}

// Test Rotation Quaternion isNear
TYPED_TEST(AngleAxisSingleTest, testIsNear){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  AngleAxis rot;

  // Check isNear
  rot = this->angleAxisGeneric1;
  ASSERT_EQ(rot.isNear(this->angleAxisGeneric1,1e-6),true);
  ASSERT_EQ(rot.isNear(this->angleAxisGeneric2,1e-6),false);
}


// --------------------------------------------------------------------------------------------------- //
// -------- Testing for casting between different type of rotations and rotation Quaternions --------- //
// --------------------------------------------------------------------------------------------------- //

// Test conversion between rotation quaternion and rotation vectors
TYPED_TEST(RotationQuaternionAngleAxisPairTest, testConversionRotationQuaternionAngleAxis){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;
  RotationQuaternion rotQuat;
  AngleAxis angleAxis;

  // TODO: add generic

  rotQuat = this->angleAxisIdentity;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-6);
  rotQuat = this->angleAxisQuarterX;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-6);
  rotQuat = this->angleAxisQuarterY;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-6);
  rotQuat = this->angleAxisQuarterZ;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-6);

  angleAxis = this->rotQuatIdentity;
  ASSERT_NEAR(angleAxis.angle(), this->angleAxisIdentity.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->angleAxisIdentity.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->angleAxisIdentity.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->angleAxisIdentity.axis().z(),1e-6);
  angleAxis = this->rotQuatQuarterX;
  ASSERT_NEAR(angleAxis.angle(), this->angleAxisQuarterX.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->angleAxisQuarterX.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->angleAxisQuarterX.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->angleAxisQuarterX.axis().z(),1e-6);
  angleAxis = this->rotQuatQuarterY;
  ASSERT_NEAR(angleAxis.angle(), this->angleAxisQuarterY.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->angleAxisQuarterY.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->angleAxisQuarterY.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->angleAxisQuarterY.axis().z(),1e-6);
  angleAxis = this->rotQuatQuarterZ;
  ASSERT_NEAR(angleAxis.angle(), this->angleAxisQuarterY.angle(),1e-6);
  ASSERT_NEAR(angleAxis.axis().x(), this->angleAxisQuarterZ.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis.axis().y(), this->angleAxisQuarterZ.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis.axis().z(), this->angleAxisQuarterZ.axis().z(),1e-6);
}

// --------------------------------------------------------------------------------------------------- //
// ------------------------------------- Testing Angle Axis ------------------------------------------ //
// --------------------------------------------------------------------------------------------------- //

// Test Rotation Vector Inversion
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


