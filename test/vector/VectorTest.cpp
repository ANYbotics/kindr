/*
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

#include "kindr/vector/VectorEigen.hpp"
#include "kindr/common/gtest_eigen.hpp"



namespace vector = kindr::vector::eigen_impl;







template <typename VectorImplementation>
struct Vector3Test: public ::testing::Test {
  typedef VectorImplementation Vector3;
  typedef typename Vector3::Scalar Scalar;
  typedef typename Vector3::Implementation EigenVector3;

  Scalar tol;
  EigenVector3 vecZero, vec1, vec2, vecAdd, vecSubtract;

  Vector3 vectorDefault;
  Vector3 vectorFromThreeValues;
  Vector3 vectorFromEigen;
  Vector3 vector2FromEigen;
  Vector3 vectorFromVector;

  Vector3Test() : tol(1e-6),
      vecZero(EigenVector3::Zero()),
      vec1(10,20,30),
      vec2(1,2,3),
      vecAdd(11,22,33),
      vecSubtract(9,18,27),
      vectorFromThreeValues(vec1.x(), vec1.y(), vec1.z()),
      vectorFromEigen(vec1),
      vector2FromEigen(vec2),
      vectorFromVector(vectorFromEigen)
  {}
};


typedef ::testing::Types<
    vector::Vector3D,
    vector::Vector3F
> Types;


TYPED_TEST_CASE(Vector3Test, Types);

TYPED_TEST(Vector3Test, testVector3)
{
  typedef typename TestFixture::Vector3 Vector3;
  typedef typename TestFixture::EigenVector3 EigenVector3;

  // default constructor
  ASSERT_EQ(this->vectorDefault.x(), this->vecZero.x()) << "Default constructor needs to initialize x-component to zero!";
  ASSERT_EQ(this->vectorDefault.y(), this->vecZero.y()) << "Default constructor needs to initialize y-component to zero!";
  ASSERT_EQ(this->vectorDefault.z(), this->vecZero.z()) << "Default constructor needs to initialize z-component to zero!";

  // constructor with three values (x,y,z)
  ASSERT_EQ(this->vectorFromThreeValues.x(), this->vec1.x()) << "Three-Value Constructor needs to first initialize x-component!";
  ASSERT_EQ(this->vectorFromThreeValues.y(), this->vec1.y()) << "Three-Value Constructor needs to second initialize y-component!";
  ASSERT_EQ(this->vectorFromThreeValues.z(), this->vec1.z()) << "Three-Value Constructor needs to third initialize z-component!";

  // constructor with Eigen vector
  ASSERT_EQ(this->vectorFromEigen.x(), this->vec1.x()) << "Base Constructor needs to first initialize x-component!";
  ASSERT_EQ(this->vectorFromEigen.y(), this->vec1.y()) << "Base Constructor needs to second initialize y-component!";
  ASSERT_EQ(this->vectorFromEigen.z(), this->vec1.z()) << "Base Constructor needs to third initialize z-component!";

  // constructor with Vector3
  ASSERT_EQ(this->vectorFromVector.x(), this->vec1.x());
  ASSERT_EQ(this->vectorFromVector.y(), this->vec1.y());
  ASSERT_EQ(this->vectorFromVector.z(), this->vec1.z());

  // toImplementation
  ASSERT_EQ(this->vectorFromThreeValues.toImplementation()(0,0), this->vec1.x()) << "X-component needs to correspond to the matrix entry (0,0)!";
  ASSERT_EQ(this->vectorFromThreeValues.toImplementation()(1,0), this->vec1.y()) << "Y-component needs to correspond to the matrix entry (1,0)!";
  ASSERT_EQ(this->vectorFromThreeValues.toImplementation()(2,0), this->vec1.z()) << "Z-component needs to correspond to the matrix entry (2,0)!";

  // addition
  Vector3 vectorAdd = this->vectorFromEigen+this->vector2FromEigen;
  ASSERT_EQ(vectorAdd.x(), this->vecAdd.x());
  ASSERT_EQ(vectorAdd.y(), this->vecAdd.y());
  ASSERT_EQ(vectorAdd.z(), this->vecAdd.z());

  // addition and assignment
  Vector3 vectorAddandAssign(this->vectorFromEigen);
  vectorAddandAssign += this->vector2FromEigen;
  ASSERT_EQ(vectorAddandAssign.x(), this->vecAdd.x());
  ASSERT_EQ(vectorAddandAssign.y(), this->vecAdd.y());
  ASSERT_EQ(vectorAddandAssign.z(), this->vecAdd.z());

  // subtract
  Vector3 vectorSubtract = this->vectorFromEigen-this->vector2FromEigen;
  ASSERT_EQ(vectorSubtract.x(), this->vecSubtract.x());
  ASSERT_EQ(vectorSubtract.y(), this->vecSubtract.y());
  ASSERT_EQ(vectorSubtract.z(), this->vecSubtract.z());

  // subtract and assignment
  Vector3 vectorSubtractandAssign(this->vectorFromEigen);
  vectorSubtractandAssign -= this->vector2FromEigen;
  ASSERT_EQ(vectorSubtractandAssign.x(), this->vecSubtract.x());
  ASSERT_EQ(vectorSubtractandAssign.y(), this->vecSubtract.y());
  ASSERT_EQ(vectorSubtractandAssign.z(), this->vecSubtract.z());

  // setZero
  Vector3 vector2FromEigenBackup(this->vec2);
  this->vector2FromEigen.setZero();
  ASSERT_EQ(this->vector2FromEigen.x(), this->vecZero.x());
  ASSERT_EQ(this->vector2FromEigen.y(), this->vecZero.y());
  ASSERT_EQ(this->vector2FromEigen.z(), this->vecZero.z());

  // bracket assign 1
  this->vector2FromEigen(0) = vector2FromEigenBackup(0);
  ASSERT_EQ(this->vector2FromEigen.x(), vector2FromEigenBackup.x());

  // << assign 2
  this->vector2FromEigen << vector2FromEigenBackup(0),vector2FromEigenBackup(1),vector2FromEigenBackup(2);
  ASSERT_EQ(this->vector2FromEigen.x(), vector2FromEigenBackup.x());
  ASSERT_EQ(this->vector2FromEigen.y(), vector2FromEigenBackup.y());
  ASSERT_EQ(this->vector2FromEigen.z(), vector2FromEigenBackup.z());

  // norm
  ASSERT_NEAR(this->vector2FromEigen.norm(), std::sqrt(this->vector2FromEigen(0)*this->vector2FromEigen(0) + this->vector2FromEigen(1)*this->vector2FromEigen(1) + this->vector2FromEigen(2)*this->vector2FromEigen(2)), 1e-6);

  // normalized
  EigenVector3 eigenVectorNormalized(this->vec2.normalized());
  Vector3 vectorNormalized(this->vector2FromEigen.normalized());
  ASSERT_NEAR(vectorNormalized.x(), eigenVectorNormalized.x(), 1e-6);
  ASSERT_NEAR(vectorNormalized.y(), eigenVectorNormalized.y(), 1e-6);
  ASSERT_NEAR(vectorNormalized.z(), eigenVectorNormalized.z(), 1e-6);

  // normalize
  this->vector2FromEigen.normalize();
  ASSERT_NEAR(this->vector2FromEigen.x(), eigenVectorNormalized.x(), 1e-6);
  ASSERT_NEAR(this->vector2FromEigen.y(), eigenVectorNormalized.y(), 1e-6);
  ASSERT_NEAR(this->vector2FromEigen.z(), eigenVectorNormalized.z(), 1e-6);

//  std::cout << vectorNormalized << std::endl;
//  std::cout << vectorNormalized.dot(vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.cross(vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.elementwiseMultiplication(vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.elementwiseDivision(vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.abs() << std::endl;
//  std::cout << vectorNormalized.max() << std::endl;
//  std::cout << vectorNormalized.min() << std::endl;
//  std::cout << (vectorNormalized == vectorNormalized) << std::endl;
//  std::cout << (vectorNormalized != vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.isSimilarTo(vectorNormalized, 1e-6) << std::endl;
//  std::cout << vectorNormalized.isSimilarTo(vector2FromEigenBackup, 1e-6) << std::endl;
//  std::cout << vectorNormalized.sum() << std::endl;
//  std::cout << vectorNormalized.mean() << std::endl;
//  std::cout << vectorNormalized.template head<2>() << std::endl;
//  std::cout << vectorNormalized.template tail<2>() << std::endl;
//    std::cout << vectorNormalized.template segment<1,2>() << std::endl;
}









template <typename VectorImplementation>
struct VectorTest: public ::testing::Test {
  typedef VectorImplementation Vector;
  typedef typename Vector::Scalar Scalar;
//  const int length = Vector::Dimension;
  typedef typename Vector::Implementation EigenVector;

  Scalar tol;
  EigenVector vecZero, vec1, vec2, vecAdd, vecSubtract;

  Vector vectorDefault;
  Vector vectorFromMultipleValues;
  Vector vectorFromEigen;
  Vector vector2FromEigen;
  Vector vectorFromVector;

  VectorTest() : tol(1e-6)
  {
    vecZero = EigenVector::Zero();
    vec1 << 10,20,30,40,50;
    vec2 << 1,2,3,4,5;
    vecAdd << 11,22,33,44,55;
    vecSubtract << 9,18,27,36,45;
    vectorFromMultipleValues << vec1(0),vec1(1),vec1(2),vec1(3),vec1(4);
    vectorFromEigen = Vector(vec1);
    vector2FromEigen = Vector(vec2);
    vectorFromVector = vectorFromEigen;
  }
};



typedef ::testing::Types<
    vector::Vector<double, 5>,
    vector::Vector<float, 5>
> Types5;


TYPED_TEST_CASE(VectorTest, Types5);

TYPED_TEST(VectorTest, testVector)
{
  typedef typename TestFixture::Vector Vector;
  typedef typename TestFixture::EigenVector EigenVector;

  // default constructor
  ASSERT_EQ(this->vectorDefault(0), this->vecZero(0)) << "Default constructor needs to initialize component 1 to zero!";
  ASSERT_EQ(this->vectorDefault(1), this->vecZero(1)) << "Default constructor needs to initialize component 2 to zero!";
  ASSERT_EQ(this->vectorDefault(2), this->vecZero(2)) << "Default constructor needs to initialize component 3 to zero!";
  ASSERT_EQ(this->vectorDefault(3), this->vecZero(3)) << "Default constructor needs to initialize component 4 to zero!";
  ASSERT_EQ(this->vectorDefault(4), this->vecZero(4)) << "Default constructor needs to initialize component 5 to zero!";

  // constructor with multiple values
  ASSERT_EQ(this->vectorFromMultipleValues(0), this->vec1(0)) << "Multi-Value Constructor needs to initialize component 1!";
  ASSERT_EQ(this->vectorFromMultipleValues(1), this->vec1(1)) << "Multi-Value Constructor needs to initialize component 2!";
  ASSERT_EQ(this->vectorFromMultipleValues(2), this->vec1(2)) << "Multi-Value Constructor needs to initialize component 3!";
  ASSERT_EQ(this->vectorFromMultipleValues(3), this->vec1(3)) << "Multi-Value Constructor needs to initialize component 4!";
  ASSERT_EQ(this->vectorFromMultipleValues(4), this->vec1(4)) << "Multi-Value Constructor needs to initialize component 5!";

  // constructor with Eigen vector
  ASSERT_EQ(this->vectorFromEigen(0), this->vec1(0)) << "Base Constructor needs to initialize component 1!";
  ASSERT_EQ(this->vectorFromEigen(1), this->vec1(1)) << "Base Constructor needs to initialize component 2!";
  ASSERT_EQ(this->vectorFromEigen(2), this->vec1(2)) << "Base Constructor needs to initialize component 3!";
  ASSERT_EQ(this->vectorFromEigen(3), this->vec1(3)) << "Base Constructor needs to initialize component 4!";
  ASSERT_EQ(this->vectorFromEigen(4), this->vec1(4)) << "Base Constructor needs to initialize component 5!";

  // constructor with Vector
  ASSERT_EQ(this->vectorFromVector(0), this->vec1(0));
  ASSERT_EQ(this->vectorFromVector(1), this->vec1(1));
  ASSERT_EQ(this->vectorFromVector(2), this->vec1(2));
  ASSERT_EQ(this->vectorFromVector(3), this->vec1(3));
  ASSERT_EQ(this->vectorFromVector(4), this->vec1(4));

  // toImplementation
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(0,0), this->vec1(0)) << "Component 1 needs to correspond to the matrix entry (0,0)!";
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(1,0), this->vec1(1)) << "Component 2 needs to correspond to the matrix entry (1,0)!";
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(2,0), this->vec1(2)) << "Component 3 needs to correspond to the matrix entry (2,0)!";
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(3,0), this->vec1(3)) << "Component 4 needs to correspond to the matrix entry (1,0)!";
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(4,0), this->vec1(4)) << "Component 5 needs to correspond to the matrix entry (2,0)!";

  // addition
  Vector vectorAdd = this->vectorFromEigen+this->vector2FromEigen;
  ASSERT_EQ(vectorAdd(0), this->vecAdd(0));
  ASSERT_EQ(vectorAdd(1), this->vecAdd(1));
  ASSERT_EQ(vectorAdd(2), this->vecAdd(2));
  ASSERT_EQ(vectorAdd(3), this->vecAdd(3));
  ASSERT_EQ(vectorAdd(4), this->vecAdd(4));

  // addition and assignment
  Vector vectorAddandAssign(this->vectorFromEigen);
  vectorAddandAssign += this->vector2FromEigen;
  ASSERT_EQ(vectorAddandAssign(0), this->vecAdd(0));
  ASSERT_EQ(vectorAddandAssign(1), this->vecAdd(1));
  ASSERT_EQ(vectorAddandAssign(2), this->vecAdd(2));
  ASSERT_EQ(vectorAddandAssign(3), this->vecAdd(3));
  ASSERT_EQ(vectorAddandAssign(4), this->vecAdd(4));

  // subtract
  Vector vectorSubtract = this->vectorFromEigen-this->vector2FromEigen;
  ASSERT_EQ(vectorSubtract(0), this->vecSubtract(0));
  ASSERT_EQ(vectorSubtract(1), this->vecSubtract(1));
  ASSERT_EQ(vectorSubtract(2), this->vecSubtract(2));
  ASSERT_EQ(vectorSubtract(3), this->vecSubtract(3));
  ASSERT_EQ(vectorSubtract(4), this->vecSubtract(4));

  // subtract and assignment
  Vector vectorSubtractandAssign(this->vectorFromEigen);
  vectorSubtractandAssign -= this->vector2FromEigen;
  ASSERT_EQ(vectorSubtractandAssign(0), this->vecSubtract(0));
  ASSERT_EQ(vectorSubtractandAssign(1), this->vecSubtract(1));
  ASSERT_EQ(vectorSubtractandAssign(2), this->vecSubtract(2));
  ASSERT_EQ(vectorSubtractandAssign(3), this->vecSubtract(3));
  ASSERT_EQ(vectorSubtractandAssign(4), this->vecSubtract(4));

  // setZero
  Vector vector2FromEigenBackup(this->vec2);
  this->vector2FromEigen.setZero();
  ASSERT_EQ(this->vector2FromEigen(0), this->vecZero(0));
  ASSERT_EQ(this->vector2FromEigen(1), this->vecZero(1));
  ASSERT_EQ(this->vector2FromEigen(2), this->vecZero(2));
  ASSERT_EQ(this->vector2FromEigen(3), this->vecZero(3));
  ASSERT_EQ(this->vector2FromEigen(4), this->vecZero(4));

  // bracket assign 1
  this->vector2FromEigen(0) = vector2FromEigenBackup(0);
  ASSERT_EQ(this->vector2FromEigen(0), vector2FromEigenBackup(0));

  // << assign 2
  this->vector2FromEigen << vector2FromEigenBackup(0),vector2FromEigenBackup(1),vector2FromEigenBackup(2),vector2FromEigenBackup(3),vector2FromEigenBackup(4);
  ASSERT_EQ(this->vector2FromEigen(0), vector2FromEigenBackup(0));
  ASSERT_EQ(this->vector2FromEigen(1), vector2FromEigenBackup(1));
  ASSERT_EQ(this->vector2FromEigen(2), vector2FromEigenBackup(2));
  ASSERT_EQ(this->vector2FromEigen(3), vector2FromEigenBackup(3));
  ASSERT_EQ(this->vector2FromEigen(4), vector2FromEigenBackup(4));

  // norm
  ASSERT_NEAR(this->vector2FromEigen.norm(), std::sqrt(this->vector2FromEigen(0)*this->vector2FromEigen(0) + this->vector2FromEigen(1)*this->vector2FromEigen(1) + this->vector2FromEigen(2)*this->vector2FromEigen(2) + this->vector2FromEigen(3)*this->vector2FromEigen(3) + this->vector2FromEigen(4)*this->vector2FromEigen(4)), 1e-6);

  // normalized
  EigenVector eigenVectorNormalized(this->vec2.normalized());
  Vector vectorNormalized(this->vector2FromEigen.normalized());
  ASSERT_NEAR(vectorNormalized(0), eigenVectorNormalized(0), 1e-6);
  ASSERT_NEAR(vectorNormalized(1), eigenVectorNormalized(1), 1e-6);
  ASSERT_NEAR(vectorNormalized(2), eigenVectorNormalized(2), 1e-6);
  ASSERT_NEAR(vectorNormalized(3), eigenVectorNormalized(3), 1e-6);
  ASSERT_NEAR(vectorNormalized(4), eigenVectorNormalized(4), 1e-6);

  // normalize
  this->vector2FromEigen.normalize();
  ASSERT_NEAR(this->vector2FromEigen(0), eigenVectorNormalized(0), 1e-6);
  ASSERT_NEAR(this->vector2FromEigen(1), eigenVectorNormalized(1), 1e-6);
  ASSERT_NEAR(this->vector2FromEigen(2), eigenVectorNormalized(2), 1e-6);
  ASSERT_NEAR(this->vector2FromEigen(3), eigenVectorNormalized(3), 1e-6);
  ASSERT_NEAR(this->vector2FromEigen(4), eigenVectorNormalized(4), 1e-6);

//  std::cout << vectorNormalized << std::endl;
//  std::cout << vectorNormalized.dot(vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.elementwiseMultiplication(vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.elementwiseDivision(vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.abs() << std::endl;
//  std::cout << vectorNormalized.max() << std::endl;
//  std::cout << vectorNormalized.min() << std::endl;
//  std::cout << (vectorNormalized == vectorNormalized) << std::endl;
//  std::cout << (vectorNormalized != vectorNormalized) << std::endl;
//  std::cout << vectorNormalized.isSimilarTo(vectorNormalized, 1e-6) << std::endl;
//  std::cout << vectorNormalized.isSimilarTo(vector2FromEigenBackup, 1e-6) << std::endl;
//  std::cout << vectorNormalized.sum() << std::endl;
//  std::cout << vectorNormalized.mean() << std::endl;
//  std::cout << vectorNormalized.template head<2>() << std::endl;
//  std::cout << vectorNormalized.template tail<2>() << std::endl;
//  std::cout << vectorNormalized.template segment<1,2>() << std::endl;
}

