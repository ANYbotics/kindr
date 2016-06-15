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

#include <gtest/gtest.h>

#include <kindr/common/assert_macros_eigen.hpp>
#include <kindr/common/gtest_eigen.hpp>
#include <kindr/quaternions/Quaternion.hpp>

namespace quat = kindr;

typedef ::testing::Types<
		quat::QuaternionD,
		quat::QuaternionF
> QuaternionTypes;

typedef ::testing::Types<
		quat::UnitQuaternionD,
		quat::UnitQuaternionF
> UnitQuaternionTypes;

typedef ::testing::Types<
		float,
		double
> PrimTypes;

typedef ::testing::Types<
	std::pair<quat::QuaternionD, quat::QuaternionD>,
	std::pair<quat::QuaternionD, quat::QuaternionF>,
	std::pair<quat::QuaternionF, quat::QuaternionF>,
	std::pair<quat::UnitQuaternionD, quat::UnitQuaternionF>,
	std::pair<quat::UnitQuaternionD, quat::QuaternionD>,
	std::pair<quat::UnitQuaternionD, quat::UnitQuaternionD>,
	std::pair<quat::UnitQuaternionF, quat::QuaternionF>,
	std::pair<quat::UnitQuaternionD, quat::QuaternionF>,
	std::pair<quat::UnitQuaternionF, quat::QuaternionD>,
	std::pair<quat::UnitQuaternionF, quat::UnitQuaternionF>
> TypePairs;

template <typename QuaternionImplementationPrimeTypePair>
struct QuaternionsPairsTest : public ::testing::Test{
  typedef typename QuaternionImplementationPrimeTypePair::first_type QuaternionFirstPrimeType;
  typedef typename QuaternionImplementationPrimeTypePair::second_type QuaternionSecondPrimeType;

  const QuaternionFirstPrimeType quat1 = QuaternionFirstPrimeType(0.0,0.36,0.48,0.8);
  const QuaternionSecondPrimeType quat2 = QuaternionSecondPrimeType(0.0,0.36,0.48,0.8);
};

template <typename PrimType>
struct QuaternionsPrimTypeTest : public ::testing::Test{
  typedef quat::Quaternion<PrimType> Quaternion;
  typedef quat::UnitQuaternion<PrimType> UnitQuaternion;

  const Quaternion quat = Quaternion(1.0,2.0,3.0,4.0);
  const UnitQuaternion uquat = UnitQuaternion(0.0,0.36,0.48,0.8);
};

template <typename QuaternionImplementation>
struct QuaternionsSingleTest : public ::testing::Test{
    typedef QuaternionImplementation Quaternion;
    typedef typename Quaternion::Scalar QuaternionScalar;
    typedef Eigen::Quaternion<QuaternionScalar> EigenQuat;

    const EigenQuat eigenQuat1 = EigenQuat(1.0,2.0,3.0,4.0);
    const EigenQuat eigenQuat2 = EigenQuat(1.23,4.56,7.89,0.12);
    const EigenQuat eigenQuat1Conj = EigenQuat(1.0,-2.0,-3.0,-4.0);
    const EigenQuat eigenQuatIdentity = EigenQuat(1.0,0.0,0.0,0.0);
    const Quaternion quat1 = Quaternion(eigenQuat1);
    const Quaternion quat2 = Quaternion(eigenQuat2);
    const Quaternion quatIdentity = Quaternion(eigenQuatIdentity);
    const QuaternionScalar norm1 = 5.477225575051661;
    const QuaternionScalar norm2 = 9.196357974763703;
};

template <typename UnitQuaternionImplementation>
struct UnitQuaternionsSingleTest : public ::testing::Test{
	  typedef UnitQuaternionImplementation UnitQuaternion;
	  typedef typename UnitQuaternion::Scalar UnitQuaternionScalar;
	  typedef Eigen::Quaternion<UnitQuaternionScalar> EigenQuat;

	  const EigenQuat eigenQuat1 = EigenQuat(0.0,0.36,0.48,0.8);
    const EigenQuat eigenQuat2 = EigenQuat(-0.48,-0.6,0.0,0.64);

    const EigenQuat eigenQuat1Conj = EigenQuat(0.0,-0.36,-0.48,-0.8);
    const EigenQuat eigenQuatIdentity = EigenQuat(1.0,0.0,0.0,0.0);
    const UnitQuaternion quat1 = UnitQuaternion(eigenQuat1);
    const UnitQuaternion quat2 = UnitQuaternion(eigenQuat2);
    const UnitQuaternion quatIdentity = UnitQuaternion(eigenQuatIdentity);
};

TYPED_TEST_CASE(QuaternionsSingleTest, QuaternionTypes);
TYPED_TEST_CASE(UnitQuaternionsSingleTest, UnitQuaternionTypes);
TYPED_TEST_CASE(QuaternionsPairsTest, TypePairs); //QuaternionsPairsTest, TypePairs
TYPED_TEST_CASE(QuaternionsPrimTypeTest, PrimTypes);

// Testing of Quaternion Constructor and Access Operator
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleConstructor) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::QuaternionScalar QuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;

  // Default constructor of quaternion needs to set all coefficients to zero
  Quaternion testQuat;
  ASSERT_EQ(testQuat.w(),QuaternionScalar(0));
  ASSERT_EQ(testQuat.x(),QuaternionScalar(0));
  ASSERT_EQ(testQuat.y(),QuaternionScalar(0));
  ASSERT_EQ(testQuat.z(),QuaternionScalar(0));

  // Constructor of quaternion using 4 scalars
  Quaternion testQuat1(this->eigenQuat1.w(), this->eigenQuat1.x(), this->eigenQuat1.y(), this->eigenQuat1.z());
  ASSERT_EQ(testQuat1.w(),this->eigenQuat1.w());
  ASSERT_EQ(testQuat1.x(),this->eigenQuat1.x());
  ASSERT_EQ(testQuat1.y(),this->eigenQuat1.y());
  ASSERT_EQ(testQuat1.z(),this->eigenQuat1.z());

  // Constructor of quaternion using real and imaginary part
  Quaternion testQuat2(this->eigenQuat1.w(),typename Quaternion::Imaginary(this->eigenQuat1.x(), this->eigenQuat1.y(), this->eigenQuat1.z()));
  ASSERT_EQ(testQuat2.w(),this->eigenQuat1.w());
  ASSERT_EQ(testQuat2.x(),this->eigenQuat1.x());
  ASSERT_EQ(testQuat2.y(),this->eigenQuat1.y());
  ASSERT_EQ(testQuat2.z(),this->eigenQuat1.z());
  ASSERT_EQ(testQuat2.real(),this->eigenQuat1.w());
  ASSERT_EQ(testQuat2.imaginary()(0),this->eigenQuat1.x());
  ASSERT_EQ(testQuat2.imaginary()(1),this->eigenQuat1.y());
  ASSERT_EQ(testQuat2.imaginary()(2),this->eigenQuat1.z());

  // Constructor of quaternion using eigen quaternion
  Quaternion testQuat3(this->eigenQuat1);
  ASSERT_EQ(testQuat3.w(),this->eigenQuat1.w());
  ASSERT_EQ(testQuat3.x(),this->eigenQuat1.x());
  ASSERT_EQ(testQuat3.y(),this->eigenQuat1.y());
  ASSERT_EQ(testQuat3.z(),this->eigenQuat1.z());
}

// Testing of Quaternion multiplication
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleMultiplication) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::QuaternionScalar QuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  Quaternion testQuat;

  // Check multiplication of two generic quaternions and compare with eigen results
  Quaternion quat12 = this->quat1*this->quat2;
  EigenQuat eigenQuat12 = this->eigenQuat1*this->eigenQuat2;
  ASSERT_EQ(quat12.w(),eigenQuat12.w());
  ASSERT_EQ(quat12.x(),eigenQuat12.x());
  ASSERT_EQ(quat12.y(),eigenQuat12.y());
  ASSERT_EQ(quat12.z(),eigenQuat12.z());

  // Check result of multiplication of a generic quaternion with identity
  testQuat = this->quat1*this->quatIdentity;
  ASSERT_EQ(testQuat.w(),this->quat1.w());
  ASSERT_EQ(testQuat.x(),this->quat1.x());
  ASSERT_EQ(testQuat.y(),this->quat1.y());
  ASSERT_EQ(testQuat.z(),this->quat1.z());
  testQuat = this->quatIdentity*this->quat1;
  ASSERT_EQ(testQuat.w(),this->quat1.w());
  ASSERT_EQ(testQuat.x(),this->quat1.x());
  ASSERT_EQ(testQuat.y(),this->quat1.y());
  ASSERT_EQ(testQuat.z(),this->quat1.z());
}

// Testing of Quaternion conjugation
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleConjugation) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::QuaternionScalar QuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  Quaternion testQuat1;
  Quaternion testQuat2;

  // Check conjugation
  testQuat1 = this->quat1;
  testQuat2 = testQuat1.conjugated();
  ASSERT_NEAR(testQuat1.w(),this->quat1.w(),1e-6);
  ASSERT_NEAR(testQuat1.x(),this->quat1.x(),1e-6);
  ASSERT_NEAR(testQuat1.y(),this->quat1.y(),1e-6);
  ASSERT_NEAR(testQuat1.z(),this->quat1.z(),1e-6);
  ASSERT_NEAR(testQuat2.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat2.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat2.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat2.z(),this->eigenQuat1Conj.z(),1e-6);
  testQuat2 = testQuat1.conjugate();
  ASSERT_NEAR(testQuat1.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat1.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat1.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat1.z(),this->eigenQuat1Conj.z(),1e-6);
  ASSERT_NEAR(testQuat2.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat2.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat2.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat2.z(),this->eigenQuat1Conj.z(),1e-6);
}

// Testing of Quaternion inversion
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleInversion) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::QuaternionScalar QuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  Quaternion testQuat;
  Quaternion testQuat1;
  Quaternion testQuat2;

  // Check inversion (the inverse is unique and the mutliplication of a quaternion with its inverse gives identity)
  testQuat1 = this->quat1.inverted();
  testQuat = testQuat1*this->quat1;
  ASSERT_NEAR(testQuat.w(),this->quatIdentity.w(),1e-6);
  ASSERT_NEAR(testQuat.x(),this->quatIdentity.x(),1e-6);
  ASSERT_NEAR(testQuat.y(),this->quatIdentity.y(),1e-6);
  ASSERT_NEAR(testQuat.z(),this->quatIdentity.z(),1e-6);
  testQuat2 = testQuat1.invert();
  ASSERT_NEAR(testQuat1.w(),this->quat1.w(),1e-6);
  ASSERT_NEAR(testQuat1.x(),this->quat1.x(),1e-6);
  ASSERT_NEAR(testQuat1.y(),this->quat1.y(),1e-6);
  ASSERT_NEAR(testQuat1.z(),this->quat1.z(),1e-6);
  ASSERT_NEAR(testQuat2.w(),this->quat1.w(),1e-6);
  ASSERT_NEAR(testQuat2.x(),this->quat1.x(),1e-6);
  ASSERT_NEAR(testQuat2.y(),this->quat1.y(),1e-6);
  ASSERT_NEAR(testQuat2.z(),this->quat1.z(),1e-6);
}

// Testing of Quaternion comparison
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleComparison) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::QuaternionScalar QuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  Quaternion testQuat;

  // Check equality comparison
  testQuat = this->quat1;
  ASSERT_EQ(testQuat==this->quat1,true);
  ASSERT_EQ(testQuat==this->quat2,false);
}

// Testing of Norm and Normalization
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleNormalization) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::QuaternionScalar QuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  Quaternion testQuat1;
  Quaternion testQuat2;
  QuaternionScalar scalar;

  // Check norm and normalization
  testQuat1 = this->quat1;
  scalar = testQuat1.norm();
  ASSERT_NEAR(scalar,this->norm1,1e-6);
  testQuat2 = testQuat1.normalized();
  ASSERT_NEAR(testQuat1.w(),this->quat1.w(),1e-6);
  ASSERT_NEAR(testQuat1.x(),this->quat1.x(),1e-6);
  ASSERT_NEAR(testQuat1.y(),this->quat1.y(),1e-6);
  ASSERT_NEAR(testQuat1.z(),this->quat1.z(),1e-6);
  ASSERT_NEAR(testQuat2.w(),this->quat1.w()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat2.x(),this->quat1.x()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat2.y(),this->quat1.y()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat2.z(),this->quat1.z()/this->norm1,1e-6);
  testQuat2 = testQuat1.normalize();
  ASSERT_NEAR(testQuat1.w(),this->quat1.w()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat1.x(),this->quat1.x()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat1.y(),this->quat1.y()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat1.z(),this->quat1.z()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat2.w(),this->quat1.w()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat2.x(),this->quat1.x()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat2.y(),this->quat1.y()/this->norm1,1e-6);
  ASSERT_NEAR(testQuat2.z(),this->quat1.z()/this->norm1,1e-6);
}

// Testing of casting to implementation
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleImplementationCast) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::QuaternionScalar QuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  EigenQuat testEigenQuat;

  // Check casting to implementation
  testEigenQuat = this->quat1.toImplementation();
  ASSERT_EQ(testEigenQuat.w(),this->eigenQuat1.w());
  ASSERT_EQ(testEigenQuat.x(),this->eigenQuat1.x());
  ASSERT_EQ(testEigenQuat.y(),this->eigenQuat1.y());
  ASSERT_EQ(testEigenQuat.z(),this->eigenQuat1.z());
}

// Testing vector() and constructor from vector
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleVectorAndVectorConstructor) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::QuaternionScalar QuaternionScalar;
  typedef Eigen::Matrix<QuaternionScalar, 4, 1> Vector4;
  Quaternion quat = this->quat1;

  // vector()
  Vector4 vector = quat.vector();
  ASSERT_NEAR(quat.w(),vector(0), 1e-3);
  ASSERT_NEAR(quat.x(),vector(1), 1e-3);
  ASSERT_NEAR(quat.y(),vector(2), 1e-3);
  ASSERT_NEAR(quat.z(),vector(3), 1e-3);

  // constructor from vector
  Quaternion quatFromVector(vector);
  ASSERT_NEAR(quat.w(),quatFromVector.w(), 1e-3);
  ASSERT_NEAR(quat.x(),quatFromVector.x(), 1e-3);
  ASSERT_NEAR(quat.y(),quatFromVector.y(), 1e-3);
  ASSERT_NEAR(quat.z(),quatFromVector.z(), 1e-3);
}

// Testing of special matrices
TYPED_TEST (QuaternionsSingleTest, testQuaternionSingleSpecialMatrices) {
  typedef typename TestFixture::Quaternion Quaternion;
  Quaternion quat1 = this->quat1;
  Quaternion quat2 = this->quat2;

  // Qleft
  Quaternion concatenation1 = quat1 * quat2;
  Quaternion concatenation2(quat1.getQuaternionMatrix() * quat2.vector());
  ASSERT_NEAR(concatenation1.w(),concatenation2.w(), 1e-3);
  ASSERT_NEAR(concatenation1.x(),concatenation2.x(), 1e-3);
  ASSERT_NEAR(concatenation1.y(),concatenation2.y(), 1e-3);
  ASSERT_NEAR(concatenation1.z(),concatenation2.z(), 1e-3);

  // Qright
  Quaternion concatenation3 = quat1 * quat2;
  Quaternion concatenation4(quat2.getConjugateQuaternionMatrix() * quat1.vector());
  ASSERT_NEAR(concatenation3.w(),concatenation4.w(), 1e-3);
  ASSERT_NEAR(concatenation3.x(),concatenation4.x(), 1e-3);
  ASSERT_NEAR(concatenation3.y(),concatenation4.y(), 1e-3);
  ASSERT_NEAR(concatenation3.z(),concatenation4.z(), 1e-3);
}

// Test simple casting (()-operator) between type and non-unit and unit quaternions
TYPED_TEST (QuaternionsPairsTest, testQuaternionPrimeTypeCasting ) {
  typedef typename TestFixture::QuaternionFirstPrimeType QuaternionFirstPrimeType;
  typedef typename TestFixture::QuaternionSecondPrimeType QuaternionSecondPrimeType;
  QuaternionFirstPrimeType testQuatFirst;
  QuaternionSecondPrimeType testQuatSecond;

  // Use ()-operator to cast between different quaternions (float and double primetype, unit and non-unit)
  // Throws an error if a non-unit quaternion is cast to a unit quaternion
  // Only tested for unit-quaternion data
  testQuatFirst(this->quat2);
  ASSERT_NEAR(testQuatFirst.w(),this->quat1.w(),1e-6);
  ASSERT_NEAR(testQuatFirst.x(),this->quat1.x(),1e-6);
  ASSERT_NEAR(testQuatFirst.y(),this->quat1.y(),1e-6);
  ASSERT_NEAR(testQuatFirst.z(),this->quat1.z(),1e-6);
  testQuatSecond(this->quat1);
  ASSERT_NEAR(testQuatSecond.w(),this->quat2.w(),1e-6);
  ASSERT_NEAR(testQuatSecond.x(),this->quat2.x(),1e-6);
  ASSERT_NEAR(testQuatSecond.y(),this->quat2.y(),1e-6);
  ASSERT_NEAR(testQuatSecond.z(),this->quat2.z(),1e-6);
}

// Test simple casting (=operator) between non-unit and unit quaternions (only unit to non-unit, no primetype casting)
TYPED_TEST (QuaternionsPrimTypeTest, testQuaternionAssignmentCasting ) {
  typename TestFixture::Quaternion testQuat;
  typename TestFixture::UnitQuaternion testUnitQuat;

  // Use =operator to cast between different quaternions (must be same primetype, unit and non-unit)
  testQuat = this->quat;
  ASSERT_EQ(testQuat.w(),this->quat.w());
  ASSERT_EQ(testQuat.x(),this->quat.x());
  ASSERT_EQ(testQuat.y(),this->quat.y());
  ASSERT_EQ(testQuat.z(),this->quat.z());
  testQuat = this->uquat;
  ASSERT_EQ(testQuat.w(),this->uquat.w());
  ASSERT_EQ(testQuat.x(),this->uquat.x());
  ASSERT_EQ(testQuat.y(),this->uquat.y());
  ASSERT_EQ(testQuat.z(),this->uquat.z());
  testUnitQuat = this->uquat;
  ASSERT_EQ(testUnitQuat.w(),this->uquat.w());
  ASSERT_EQ(testUnitQuat.x(),this->uquat.x());
  ASSERT_EQ(testUnitQuat.y(),this->uquat.y());
  ASSERT_EQ(testUnitQuat.z(),this->uquat.z());
}

// Test toUnitquaternion explicit casting
TYPED_TEST (QuaternionsPrimTypeTest, testQuaternionToUnitQuaternion ) {
  typename TestFixture::Quaternion testQuat;
  typename TestFixture::UnitQuaternion testUnitQuat;

  // Use toUnitQuaternion method in order normalize and cast a Quaternion to a unit Quaternion
  testUnitQuat = this->quat.toUnitQuaternion(); // w is 1.0
  ASSERT_NEAR(testUnitQuat.norm(),1.0,1e-6);
  ASSERT_NEAR(testUnitQuat.x()/testUnitQuat.w(),this->quat.x(),1e-6);
  ASSERT_NEAR(testUnitQuat.y()/testUnitQuat.w(),this->quat.y(),1e-6);
  ASSERT_NEAR(testUnitQuat.z()/testUnitQuat.w(),this->quat.z(),1e-6);
}

// Testing of UnitQuaternion Constructor and Access Operator
TYPED_TEST (UnitQuaternionsSingleTest, testUnitQuaternionSingleConstructor) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::UnitQuaternionScalar UnitQuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;

  // Default constructor of quaternion needs to set all coefficients to zero
  UnitQuaternion testQuat;
  ASSERT_EQ(testQuat.w(),UnitQuaternionScalar(1));
  ASSERT_EQ(testQuat.x(),UnitQuaternionScalar(0));
  ASSERT_EQ(testQuat.y(),UnitQuaternionScalar(0));
  ASSERT_EQ(testQuat.z(),UnitQuaternionScalar(0));

  // Constructor of quaternion using 4 scalars
  UnitQuaternion testQuat1(this->eigenQuat1.w(), this->eigenQuat1.x(), this->eigenQuat1.y(), this->eigenQuat1.z());
  ASSERT_EQ(testQuat1.w(),this->eigenQuat1.w());
  ASSERT_EQ(testQuat1.x(),this->eigenQuat1.x());
  ASSERT_EQ(testQuat1.y(),this->eigenQuat1.y());
  ASSERT_EQ(testQuat1.z(),this->eigenQuat1.z());

  // Constructor of quaternion using real and imaginary part
  UnitQuaternion testQuat2(this->eigenQuat1.w(),typename UnitQuaternion::Imaginary(this->eigenQuat1.x(), this->eigenQuat1.y(), this->eigenQuat1.z()));
  ASSERT_EQ(testQuat2.w(),this->eigenQuat1.w());
  ASSERT_EQ(testQuat2.x(),this->eigenQuat1.x());
  ASSERT_EQ(testQuat2.y(),this->eigenQuat1.y());
  ASSERT_EQ(testQuat2.z(),this->eigenQuat1.z());
  ASSERT_EQ(testQuat2.real(),this->eigenQuat1.w());
  ASSERT_EQ(testQuat2.imaginary()(0),this->eigenQuat1.x());
  ASSERT_EQ(testQuat2.imaginary()(1),this->eigenQuat1.y());
  ASSERT_EQ(testQuat2.imaginary()(2),this->eigenQuat1.z());

  // Constructor of quaternion using eigen quaternion
  UnitQuaternion testQuat3(this->eigenQuat1);
  ASSERT_EQ(testQuat3.w(),this->eigenQuat1.w());
  ASSERT_EQ(testQuat3.x(),this->eigenQuat1.x());
  ASSERT_EQ(testQuat3.y(),this->eigenQuat1.y());
  ASSERT_EQ(testQuat3.z(),this->eigenQuat1.z());
}

// Testing of UnitQuaternion multiplication
TYPED_TEST (UnitQuaternionsSingleTest, testUnitQuaternionSingleMultiplication) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::UnitQuaternionScalar UnitQuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  UnitQuaternion testQuat;

  // Check multiplication of two generic quaternions and compare with eigen results
  UnitQuaternion quat12 = this->quat1*this->quat2;
  EigenQuat eigenQuat12 = this->eigenQuat1*this->eigenQuat2;
  ASSERT_EQ(quat12.w(),eigenQuat12.w());
  ASSERT_EQ(quat12.x(),eigenQuat12.x());
  ASSERT_EQ(quat12.y(),eigenQuat12.y());
  ASSERT_EQ(quat12.z(),eigenQuat12.z());

  // Check result of multiplication of a generic quaternion with identity
  testQuat = this->quat1*this->quatIdentity;
  ASSERT_EQ(testQuat.w(),this->quat1.w());
  ASSERT_EQ(testQuat.x(),this->quat1.x());
  ASSERT_EQ(testQuat.y(),this->quat1.y());
  ASSERT_EQ(testQuat.z(),this->quat1.z());
  testQuat = this->quatIdentity*this->quat1;
  ASSERT_EQ(testQuat.w(),this->quat1.w());
  ASSERT_EQ(testQuat.x(),this->quat1.x());
  ASSERT_EQ(testQuat.y(),this->quat1.y());
  ASSERT_EQ(testQuat.z(),this->quat1.z());
}

// Testing of UnitQuaternion conjugation
TYPED_TEST (UnitQuaternionsSingleTest, testUnitQuaternionSingleConjugation) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::UnitQuaternionScalar UnitQuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  UnitQuaternion testQuat1;
  UnitQuaternion testQuat2;

  // Check conjugation
  testQuat1 = this->quat1;
  testQuat2 = testQuat1.conjugated();
  ASSERT_NEAR(testQuat1.w(),this->quat1.w(),1e-6);
  ASSERT_NEAR(testQuat1.x(),this->quat1.x(),1e-6);
  ASSERT_NEAR(testQuat1.y(),this->quat1.y(),1e-6);
  ASSERT_NEAR(testQuat1.z(),this->quat1.z(),1e-6);
  ASSERT_NEAR(testQuat2.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat2.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat2.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat2.z(),this->eigenQuat1Conj.z(),1e-6);
  testQuat2 = testQuat1.conjugate();
  ASSERT_NEAR(testQuat1.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat1.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat1.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat1.z(),this->eigenQuat1Conj.z(),1e-6);
  ASSERT_NEAR(testQuat2.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat2.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat2.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat2.z(),this->eigenQuat1Conj.z(),1e-6);
}

// Testing of UnitQuaternion inversion
TYPED_TEST (UnitQuaternionsSingleTest, testUnitQuaternionSingleInversion) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::UnitQuaternionScalar UnitQuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  UnitQuaternion testQuat1;
  UnitQuaternion testQuat2;

  // Check conjugation
  testQuat1 = this->quat1;
  testQuat2 = testQuat1.inverted();
  ASSERT_NEAR(testQuat1.w(),this->quat1.w(),1e-6);
  ASSERT_NEAR(testQuat1.x(),this->quat1.x(),1e-6);
  ASSERT_NEAR(testQuat1.y(),this->quat1.y(),1e-6);
  ASSERT_NEAR(testQuat1.z(),this->quat1.z(),1e-6);
  ASSERT_NEAR(testQuat2.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat2.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat2.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat2.z(),this->eigenQuat1Conj.z(),1e-6);
  testQuat2 = testQuat1.invert();
  ASSERT_NEAR(testQuat1.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat1.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat1.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat1.z(),this->eigenQuat1Conj.z(),1e-6);
  ASSERT_NEAR(testQuat2.w(),this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(testQuat2.x(),this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(testQuat2.y(),this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(testQuat2.z(),this->eigenQuat1Conj.z(),1e-6);
}

// Testing of UnitQuaternion comparison
TYPED_TEST (UnitQuaternionsSingleTest, testUnitQuaternionSingleComparison) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::UnitQuaternionScalar UnitQuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  UnitQuaternion testQuat;

  // Check equality comparison
  testQuat = this->quat1;
  ASSERT_EQ(testQuat==this->quat1,true);
  ASSERT_EQ(testQuat==this->quat2,false);
}

// Testing of Norm and Normalization for UnitQuaternion
TYPED_TEST (UnitQuaternionsSingleTest, testUnitQuaternionSingleNormalization) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::UnitQuaternionScalar UnitQuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  UnitQuaternion testQuat;
  UnitQuaternionScalar scalar;

  // Check norm and normalization
  testQuat = this->quat1;
  scalar = testQuat.norm();
  ASSERT_NEAR(scalar,1.0,1e-6);
}

// Testing of casting to implementation for UnitQuaternion
TYPED_TEST (UnitQuaternionsSingleTest, testUnitQuaternionSingleImplementationCast) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::UnitQuaternionScalar UnitQuaternionScalar;
  typedef typename TestFixture::EigenQuat EigenQuat;
  EigenQuat testEigenQuat;

  // Check casting to implementation
  testEigenQuat = this->quat1.toImplementation();
  ASSERT_EQ(testEigenQuat.w(),this->eigenQuat1.w());
  ASSERT_EQ(testEigenQuat.x(),this->eigenQuat1.x());
  ASSERT_EQ(testEigenQuat.y(),this->eigenQuat1.y());
  ASSERT_EQ(testEigenQuat.z(),this->eigenQuat1.z());
}

// Testing of special matrices
TYPED_TEST (UnitQuaternionsSingleTest, testUnitQuaternionSingleSpecialMatrices) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  UnitQuaternion quat1 = this->quat1;
  UnitQuaternion quat2 = this->quat2;

  // Qleft
  UnitQuaternion concatenation1 = quat1 * quat2;
  UnitQuaternion concatenation2(quat1.getQuaternionMatrix() * quat2.vector());
  ASSERT_NEAR(concatenation1.w(),concatenation2.w(), 1e-3);
  ASSERT_NEAR(concatenation1.x(),concatenation2.x(), 1e-3);
  ASSERT_NEAR(concatenation1.y(),concatenation2.y(), 1e-3);
  ASSERT_NEAR(concatenation1.z(),concatenation2.z(), 1e-3);

  // Qright
  UnitQuaternion concatenation3 = quat1 * quat2;
  UnitQuaternion concatenation4(quat2.getConjugateQuaternionMatrix() * quat1.vector());
  ASSERT_NEAR(concatenation3.w(),concatenation4.w(), 1e-3);
  ASSERT_NEAR(concatenation3.x(),concatenation4.x(), 1e-3);
  ASSERT_NEAR(concatenation3.y(),concatenation4.y(), 1e-3);
  ASSERT_NEAR(concatenation3.z(),concatenation4.z(), 1e-3);
}

// Testing vector() and constructor from vector
TYPED_TEST (UnitQuaternionsSingleTest, testQuaternionSingleVectorAndVectorConstructor) {
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::UnitQuaternionScalar UnitQuaternionScalar;
  typedef Eigen::Matrix<UnitQuaternionScalar, 4, 1> Vector4;
  UnitQuaternion quat = this->quat1;

  // vector()
  Vector4 vector = quat.vector();
  ASSERT_NEAR(quat.w(),vector(0), 1e-3);
  ASSERT_NEAR(quat.x(),vector(1), 1e-3);
  ASSERT_NEAR(quat.y(),vector(2), 1e-3);
  ASSERT_NEAR(quat.z(),vector(3), 1e-3);

  // constructor from vector
  UnitQuaternion quatFromVector(vector);
  ASSERT_NEAR(quat.w(),quatFromVector.w(), 1e-3);
  ASSERT_NEAR(quat.x(),quatFromVector.x(), 1e-3);
  ASSERT_NEAR(quat.y(),quatFromVector.y(), 1e-3);
  ASSERT_NEAR(quat.z(),quatFromVector.z(), 1e-3);
}



