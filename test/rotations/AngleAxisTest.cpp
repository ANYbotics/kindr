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

#include "kindr/rotations/Rotation.hpp"
#include "kindr/common/gtest_eigen.hpp"
#include "kindr/quaternions/Quaternion.hpp"

namespace rot = kindr;
namespace quat = kindr;

using namespace kindr;

template <typename Rotation_>
class AngleAxisSingleTest : public ::testing::Test{
 public:
  typedef Rotation_ AngleAxis;
  typedef typename Rotation_::Scalar Scalar;
  typedef Eigen::Matrix<Scalar,4,1> Vector4;
  typedef Eigen::Matrix<Scalar,3,1> Vector;
  typedef typename rot::RotationQuaternion<Scalar> RotationQuaternion;
  Vector4 eigenVector4Identity;
  Vector4 eigenVector4v1;
  Eigen::AngleAxis<Scalar> eigenAngleAxis;

  const AngleAxis angleAxisGeneric1 = AngleAxis(0.2, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
  const AngleAxis angleAxisGeneric1Plus2Pi = AngleAxis(0.2+2.0*M_PI, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
  const AngleAxis angleAxisGeneric1Minus2Pi = AngleAxis(0.2-2.0*M_PI, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
  const AngleAxis angleAxisGeneric2 = AngleAxis(0.6, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));

  const AngleAxis rotAngleAxisQuarterX = AngleAxis(M_PI/2.0, 1.0, 0.0, 0.0);
  const AngleAxis rotAngleAxisQuarterY = AngleAxis(M_PI/2.0, 0.0, 1.0, 0.0);
  const AngleAxis rotAngleAxisQuarterZ = AngleAxis(M_PI/2.0, 0.0, 0.0, 1.0);
  const AngleAxis rotAngleAxisIdentity = AngleAxis(0.0, 1.0, 0.0, 0.0);
  const Vector vec = Vector(0.3,-1.5,0.6);
  const Vector vecX = Vector(1.0,0.0,0.0);
  const Vector vecXSmallError = Vector(1.0000001,0.0,0.0);
  const Vector vecY = Vector(0.0,1.0,0.0);
  const Vector vecZ = Vector(0.0,0.0,1.0);

  AngleAxis rotAngleAxis1;
  AngleAxis rotAngleAxis2;

  AngleAxisSingleTest() {
    eigenVector4Identity << 0.0, 1.0, 0.0, 0.0;
    eigenVector4v1 << 1.2, 2.0/sqrt(4+9+16), 3.0/sqrt(4+9+16), 4.0/sqrt(4+9+16);
    eigenAngleAxis.angle() = eigenVector4v1(0);
    eigenAngleAxis.axis() = eigenVector4v1.template block<3,1>(1,0);


    rotAngleAxis1 = AngleAxis(RotationQuaternion(0.0,0.36,0.48,0.8));
    rotAngleAxis2 = AngleAxis(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)));
  }
};

template <typename RotationPair_>
struct RotationQuaternionAngleAxisPairTest : public ::testing::Test{
  typedef typename RotationPair_::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationPair_::second_type AngleAxis;
  typedef typename AngleAxis::Scalar AngleAxisScalar;
  typedef Eigen::Matrix<RotationQuaternionScalar,3,1> Vector;

  const RotationQuaternionScalar rotQuatSmallNumber = kindr::internal::NumTraits<RotationQuaternionScalar>::dummy_precision()/10.0;
  const RotationQuaternionScalar angleAxisSmallNumber = kindr::internal::NumTraits<AngleAxisScalar>::dummy_precision()/10.0;

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

template <typename ImplementationPairs_>
struct AngleAxisActiveTest : public RotationQuaternionAngleAxisPairTest<ImplementationPairs_>{

};

template <typename ImplementationPairs_>
struct AngleAxisPassiveTest : public RotationQuaternionAngleAxisPairTest<ImplementationPairs_>{
};


typedef ::testing::Types<
    rot::AngleAxisPD,
    rot::AngleAxisPF
> AngleAxisTypes;

typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPD, rot::AngleAxisPD>,
    std::pair<rot::RotationQuaternionPF, rot::AngleAxisPF>
> AngleAxisPassiveTypes;



typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPF, rot::AngleAxisPF>,
    std::pair<rot::RotationQuaternionPF, rot::AngleAxisPD>,
    std::pair<rot::RotationQuaternionPD, rot::AngleAxisPF>,
    std::pair<rot::RotationQuaternionPD, rot::AngleAxisPD>
> TypeQuaternionAngleAxisPairs;

TYPED_TEST_CASE(AngleAxisSingleTest, AngleAxisTypes);
TYPED_TEST_CASE(RotationQuaternionAngleAxisPairTest, TypeQuaternionAngleAxisPairs);
TYPED_TEST_CASE(AngleAxisPassiveTest, AngleAxisPassiveTypes);




// Testing constructors
TYPED_TEST(AngleAxisSingleTest, testConstructors){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;

  AngleAxis rot;
  ASSERT_EQ(this->eigenVector4Identity(0), rot.angle());
  ASSERT_EQ(this->eigenVector4Identity(1), rot.axis().x());
  ASSERT_EQ(this->eigenVector4Identity(2), rot.axis().y());
  ASSERT_EQ(this->eigenVector4Identity(3), rot.axis().z());

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
  typedef typename TestFixture::Vector Vector;

  AngleAxis rot(this->eigenVector4v1(0), this->eigenVector4v1(1), this->eigenVector4v1(2), this->eigenVector4v1(3));
  rot.setIdentity();
  ASSERT_EQ(this->eigenVector4Identity(0), rot.angle());
  ASSERT_EQ(this->eigenVector4Identity(1), rot.axis().x());
  ASSERT_EQ(this->eigenVector4Identity(2), rot.axis().y());
  ASSERT_EQ(this->eigenVector4Identity(3), rot.axis().z());

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

  rot.setFromVectors(this->vec, this->vec);
  ASSERT_NEAR(rot.angle(), this->rotAngleAxisIdentity.angle(),1e-6);
  ASSERT_NEAR(rot.axis().x(), this->rotAngleAxisIdentity.axis().x(),1e-6);
  ASSERT_NEAR(rot.axis().y(), this->rotAngleAxisIdentity.axis().y(),1e-6);
  ASSERT_NEAR(rot.axis().z(), this->rotAngleAxisIdentity.axis().z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecY);
  ASSERT_NEAR(rot.angle(), this->rotAngleAxisQuarterZ.angle(),1e-6);
  ASSERT_NEAR(rot.axis().x(), this->rotAngleAxisQuarterZ.axis().x(),1e-6);
  ASSERT_NEAR(rot.axis().y(), this->rotAngleAxisQuarterZ.axis().y(),1e-6);
  ASSERT_NEAR(rot.axis().z(), this->rotAngleAxisQuarterZ.axis().z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecX);
  ASSERT_NEAR(rot.angle(), this->rotAngleAxisIdentity.angle(),1e-6);
  ASSERT_NEAR(rot.axis().x(), this->rotAngleAxisIdentity.axis().x(),1e-6);
  ASSERT_NEAR(rot.axis().y(), this->rotAngleAxisIdentity.axis().y(),1e-6);
  ASSERT_NEAR(rot.axis().z(), this->rotAngleAxisIdentity.axis().z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecXSmallError);
  ASSERT_NEAR(rot.angle(), this->rotAngleAxisIdentity.angle(),1e-6);
  ASSERT_NEAR(rot.axis().x(), this->rotAngleAxisIdentity.axis().x(),1e-6);
  ASSERT_NEAR(rot.axis().y(), this->rotAngleAxisIdentity.axis().y(),1e-6);
  ASSERT_NEAR(rot.axis().z(), this->rotAngleAxisIdentity.axis().z(),1e-6);



}


// Test comparison (equality)
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

// Test comparison (inequality)
TYPED_TEST(AngleAxisSingleTest, testComparisonNotEqual){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  AngleAxis rot;

  // Check equality comparison
  rot = this->angleAxisGeneric1;
  ASSERT_EQ(false, rot!=this->angleAxisGeneric1);
  ASSERT_EQ(false, rot!=this->angleAxisGeneric1Plus2Pi);
  ASSERT_EQ(false, rot!=this->angleAxisGeneric1Minus2Pi);
  ASSERT_EQ(true, rot!=this->angleAxisGeneric2);
}

// Test  getDisparityAngle
TYPED_TEST(AngleAxisSingleTest, testGetDisparityAngle){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  ASSERT_NEAR(this->rotAngleAxis1.getDisparityAngle(this->rotAngleAxis1),0.0,1e-6);
  ASSERT_NEAR(this->rotAngleAxis2.getDisparityAngle(this->rotAngleAxis2),0.0,1e-6);
  ASSERT_NEAR(this->rotAngleAxisIdentity.getDisparityAngle(this->rotAngleAxisIdentity),0.0,1e-6);
  ASSERT_NEAR(this->rotAngleAxis2.getDisparityAngle(this->rotAngleAxis1),this->rotAngleAxis1.getDisparityAngle(this->rotAngleAxis2),1e-6);
  ASSERT_NEAR(this->rotAngleAxis1.getDisparityAngle(this->rotAngleAxisIdentity),calcRotationQuatDisparityAngleToIdentity(RotationQuaternion(this->rotAngleAxis1)),1e-6);
  ASSERT_NEAR(this->rotAngleAxis2.getDisparityAngle(this->rotAngleAxis1),calcRotationQuatDisparityAngle(RotationQuaternion(this->rotAngleAxis1), RotationQuaternion(this->rotAngleAxis2)),1e-6);
}


// Test isNear
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



// Test Concatenation
TYPED_TEST(AngleAxisSingleTest, testConcatenation){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  AngleAxis rotAngleAxis;

  // Check result of multiplication of a generic rotation with identity
  rotAngleAxis = this->rotAngleAxis1*this->rotAngleAxisIdentity;
  ASSERT_EQ(rotAngleAxis==this->rotAngleAxis1,true);
  rotAngleAxis = this->rotAngleAxisIdentity*this->rotAngleAxis1;
  ASSERT_EQ(rotAngleAxis==this->rotAngleAxis1,true);

  // Check concatenation of 4 quarters
  rotAngleAxis = this->rotAngleAxisQuarterX*this->rotAngleAxisQuarterX*this->rotAngleAxisQuarterX*this->rotAngleAxisQuarterX;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisIdentity.getUnique().angle(),1e-6);
  // Axis is undefined if rotation near 0 -> no test necessary
  rotAngleAxis = this->rotAngleAxisQuarterY*this->rotAngleAxisQuarterY*this->rotAngleAxisQuarterY*this->rotAngleAxisQuarterY;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisIdentity.getUnique().angle(),1e-6);
// Axis is undefined if rotation near 0 -> no test necessary
  rotAngleAxis = this->rotAngleAxisQuarterZ*this->rotAngleAxisQuarterZ*this->rotAngleAxisQuarterZ*this->rotAngleAxisQuarterZ;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisIdentity.getUnique().angle(),1e-6);
// Axis is undefined if rotation near 0 -> no test necessary

  // Check concatenation of 3 different quarters
  rotAngleAxis = this->rotAngleAxisQuarterX.inverted()*this->rotAngleAxisQuarterY*this->rotAngleAxisQuarterX;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisQuarterZ.inverted().getUnique().angle(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().x(), this->rotAngleAxisQuarterZ.inverted().getUnique().axis().x(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().y(), this->rotAngleAxisQuarterZ.inverted().getUnique().axis().y(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().z(), this->rotAngleAxisQuarterZ.inverted().getUnique().axis().z(),1e-6);
  rotAngleAxis = this->rotAngleAxisQuarterX.inverted()*this->rotAngleAxisQuarterZ*this->rotAngleAxisQuarterX;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisQuarterY.getUnique().angle(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().x(), this->rotAngleAxisQuarterY.getUnique().axis().x(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().y(), this->rotAngleAxisQuarterY.getUnique().axis().y(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().z(), this->rotAngleAxisQuarterY.getUnique().axis().z(),1e-6);
  rotAngleAxis = this->rotAngleAxisQuarterY.inverted()*this->rotAngleAxisQuarterX*this->rotAngleAxisQuarterY;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisQuarterZ.getUnique().angle(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().x(), this->rotAngleAxisQuarterZ.getUnique().axis().x(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().y(), this->rotAngleAxisQuarterZ.getUnique().axis().y(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().z(), this->rotAngleAxisQuarterZ.getUnique().axis().z(),1e-6);
  rotAngleAxis = this->rotAngleAxisQuarterY.inverted()*this->rotAngleAxisQuarterZ*this->rotAngleAxisQuarterY;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisQuarterX.inverted().getUnique().angle(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().x(), this->rotAngleAxisQuarterX.inverted().getUnique().axis().x(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().y(), this->rotAngleAxisQuarterX.inverted().getUnique().axis().y(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().z(), this->rotAngleAxisQuarterX.inverted().getUnique().axis().z(),1e-6);
  rotAngleAxis = this->rotAngleAxisQuarterZ.inverted()*this->rotAngleAxisQuarterX*this->rotAngleAxisQuarterZ;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisQuarterY.inverted().getUnique().angle(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().x(), this->rotAngleAxisQuarterY.inverted().getUnique().axis().x(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().y(), this->rotAngleAxisQuarterY.inverted().getUnique().axis().y(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().z(), this->rotAngleAxisQuarterY.inverted().getUnique().axis().z(),1e-6);
  rotAngleAxis = this->rotAngleAxisQuarterZ.inverted()*this->rotAngleAxisQuarterY*this->rotAngleAxisQuarterZ;
  ASSERT_NEAR(rotAngleAxis.getUnique().angle(), this->rotAngleAxisQuarterX.getUnique().angle(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().x(), this->rotAngleAxisQuarterX.getUnique().axis().x(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().y(), this->rotAngleAxisQuarterX.getUnique().axis().y(),1e-6);
  ASSERT_NEAR(rotAngleAxis.getUnique().axis().z(), this->rotAngleAxisQuarterX.getUnique().axis().z(),1e-6);
}


// Test fix
TYPED_TEST(AngleAxisSingleTest, testFix){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  AngleAxis rot;
  AngleAxis rotModified;

  // Check fix
  rot = this->angleAxisGeneric1;
  rotModified = rot;
  rotModified.toImplementation().axis() *= 1.1;
  rotModified.fix();
  ASSERT_NEAR(rotModified.angle(), rot.angle(),1e-6);
  ASSERT_NEAR(rotModified.axis().x(), rot.axis().x(),1e-6);
  ASSERT_NEAR(rotModified.axis().y(), rot.axis().y(),1e-6);
  ASSERT_NEAR(rotModified.axis().z(), rot.axis().z(),1e-6);
}



// Test Vector Rotation
TYPED_TEST(AngleAxisSingleTest, testVectorRotation){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  AngleAxis rotQuat;
  Vector testVec;
  Vector testVec1;
  Vector testVec2;

  int signSwitch = 1;

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



/* Test Exponential and Logarithmic Map
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(AngleAxisSingleTest, testMaps){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;

  testVec = this->rotAngleAxisIdentity.logarithmicMap();
  ASSERT_NEAR(testVec(0), 0.0,1e-6);
  ASSERT_NEAR(testVec(1), 0.0,1e-6);
  ASSERT_NEAR(testVec(2), 0.0,1e-6);

  testVec = this->rotAngleAxis1.logarithmicMap();
  AngleAxis rotAngleAxisExpMap = AngleAxis::exponentialMap(testVec);
  ASSERT_EQ(rotAngleAxisExpMap.isNear(this->rotAngleAxis1,1e-6),true);

  testVec = this->rotAngleAxis2.logarithmicMap();
  rotAngleAxisExpMap =  AngleAxis::exponentialMap(testVec);
  ASSERT_EQ(rotAngleAxisExpMap.isNear(this->rotAngleAxis2,1e-6),true);

  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rotAngleAxisExpMap = AngleAxis::exponentialMap(testVec);
  ASSERT_NEAR(rotAngleAxisExpMap.getDisparityAngle(this->rotAngleAxisIdentity),norm,1e-6);

  testVec.setZero();
  rotAngleAxisExpMap = AngleAxis::exponentialMap(testVec);
  ASSERT_NEAR(rotAngleAxisExpMap.angle(), this->rotAngleAxisIdentity.angle(),1e-6);
  ASSERT_NEAR(rotAngleAxisExpMap.axis().x(), this->rotAngleAxisIdentity.axis().x(),1e-6);
  ASSERT_NEAR(rotAngleAxisExpMap.axis().y(), this->rotAngleAxisIdentity.axis().y(),1e-6);
  ASSERT_NEAR(rotAngleAxisExpMap.axis().z(), this->rotAngleAxisIdentity.axis().z(),1e-6);
}


/*  Test Box Operations
 * Assumes isNear() of Angle Axis is correct.
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(AngleAxisSingleTest, testBoxOperators){
  typedef typename TestFixture::AngleAxis AngleAxis;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  AngleAxis rotAngleAxis;
  AngleAxis rotAngleAxis2;
  Vector testVec;

  // Test addition with 0
  testVec.setZero();
  rotAngleAxis = this->rotAngleAxis1.boxPlus(testVec);
  ASSERT_EQ(rotAngleAxis.isNear(this->rotAngleAxis1,1e-6),true);

  // Test subtraction of same elements
  testVec = this->rotAngleAxis1.boxMinus(this->rotAngleAxis1);
  ASSERT_NEAR(testVec(0),0.0,1e-6);
  ASSERT_NEAR(testVec(1),0.0,1e-6);
  ASSERT_NEAR(testVec(2),0.0,1e-6);

  // Test backward-forward
  testVec = this->rotAngleAxis1.boxMinus(this->rotAngleAxis2);
  rotAngleAxis = this->rotAngleAxis2.boxPlus(testVec);
  ASSERT_EQ(rotAngleAxis.isNear(this->rotAngleAxis1,1e-6),true);

  // Test forward-backward
  testVec = this->vec;
  rotAngleAxis = this->rotAngleAxis1.boxPlus(testVec);
  testVec = rotAngleAxis.boxMinus(this->rotAngleAxis1);
  ASSERT_NEAR(testVec(0),this->vec(0),1e-6);
  ASSERT_NEAR(testVec(1),this->vec(1),1e-6);
  ASSERT_NEAR(testVec(2),this->vec(2),1e-6);

  // Test overlap with disparity angle
  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rotAngleAxis = this->rotAngleAxis1.boxPlus(testVec);
  ASSERT_NEAR(rotAngleAxis.getDisparityAngle(this->rotAngleAxis1),norm,1e-4); // Check distance between both
  rotAngleAxis2 = this->rotAngleAxis1.boxPlus(2*testVec);
  ASSERT_NEAR(rotAngleAxis.getDisparityAngle(rotAngleAxis2),norm,1e-4); // Check distance to double
  rotAngleAxis2 = this->rotAngleAxis1.boxPlus(-testVec);
  ASSERT_NEAR(rotAngleAxis.getDisparityAngle(rotAngleAxis2),2*norm,1e-4); // Check distance to reverse
}



// --------------------------------------------------------------------------------------------------- //
// ---------------------------- RotationQuaternionAngleAxisPairTest ---------------------------------- //
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


/* Test inversion
 * Assumes inversion of RotationQuaternion is correct.
 * Assumes conversion between AngleAxis and RotationQuaternion is correct.
 */
TYPED_TEST(RotationQuaternionAngleAxisPairTest, testAngleAxisInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::AngleAxis AngleAxis;
  RotationQuaternion rotQuat;
  AngleAxis angleAxis1;
  AngleAxis angleAxis2;
  AngleAxis angleAxis3;
  AngleAxis angleAxis4;
  AngleAxis angleAxis5;

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
  angleAxis5 = this->rotQuat1;
  angleAxis4 = angleAxis3.inverted();
  ASSERT_NEAR(angleAxis1.angle(),angleAxis4.angle(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().x(),angleAxis4.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().y(),angleAxis4.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis1.axis().z(),angleAxis4.axis().z(),1e-6);

  ASSERT_NEAR(angleAxis3.angle(),angleAxis5.angle(),1e-6);
  ASSERT_NEAR(angleAxis3.axis().x(),angleAxis5.axis().x(),1e-6);
  ASSERT_NEAR(angleAxis3.axis().y(),angleAxis5.axis().y(),1e-6);
  ASSERT_NEAR(angleAxis3.axis().z(),angleAxis5.axis().z(),1e-6);
}
