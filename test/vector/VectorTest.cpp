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
#include "kindr/rotations/RotationEigen.hpp"
#include "kindr/common/gtest_eigen.hpp"



namespace vector = kindr::vector::eigen_impl;
namespace rotations = kindr::rotations::eigen_impl;







template <typename VectorImplementation>
struct VectorTest: public ::testing::Test {
  typedef VectorImplementation Vector;
  typedef typename Vector::Scalar Scalar;
//  const int length = Vector::Dimension;
  typedef typename Vector::Implementation EigenVector;

  Scalar tol, sum, max, min, mean;
  EigenVector vecZero, vec1, vec2, vecAdd, vecSubtract;

  Vector vectorDefault;
  Vector vectorFromMultipleValues;
  Vector vector1FromEigen;
  Vector vector2FromEigen;
  Vector vectorFromVector;

  VectorTest() : tol(1e-6),
                 sum(150),
                 max(50),
                 min(10),
                 mean(30)
  {
    vecZero = EigenVector::Zero();
    vec1 << 10,20,30,40,50;
    vec2 << 1,2,3,4,5;
    vecAdd << 11,22,33,44,55;
    vecSubtract << 9,18,27,36,45;
    vectorFromMultipleValues << vec1(0),vec1(1),vec1(2),vec1(3),vec1(4);
    vector1FromEigen = Vector(vec1);
    vector2FromEigen = Vector(vec2);
    vectorFromVector = vector1FromEigen;
  }
};



typedef ::testing::Types<
    vector::Vector<kindr::phys_quant::PhysicalType::None, double, 5>,
    vector::Vector<kindr::phys_quant::PhysicalType::None, float,  5>,
    vector::Vector<kindr::phys_quant::PhysicalType::Force, double, 5>,
    vector::Vector<kindr::phys_quant::PhysicalType::Force, float,  5>,
    vector::Vector<kindr::phys_quant::PhysicalType::Length, double, 5>,
    vector::Vector<kindr::phys_quant::PhysicalType::Length, float,  5>
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

