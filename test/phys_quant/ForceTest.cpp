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

#include "kindr/phys_quant/ForceEigen.hpp"
#include "kindr/common/gtest_eigen.hpp"



namespace force = kindr::phys_quant::eigen_impl;







template <typename ForceImplementation>
struct Force3Test: public ::testing::Test {
  typedef ForceImplementation Force3;
  typedef typename Force3::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> EigenVector3;

  Scalar tol;
  EigenVector3 vecZero, vec1, vec2, vecAdd, vecSubtract;

  Force3 forceDefault;
  Force3 forceFromThreeValues;
  Force3 forceFromEigen;
  Force3 force2FromEigen;
  Force3 forceFromForce;

  Force3Test() : tol(1e-6),
      vecZero(EigenVector3::Zero()),
      vec1(10,20,30),
      vec2(1,2,3),
      vecAdd(11,22,33),
      vecSubtract(9,18,27),
      forceFromThreeValues(vec1.x(), vec1.y(), vec1.z()),
      forceFromEigen(vec1),
      force2FromEigen(vec2),
      forceFromForce(forceFromEigen)
  {}
};


typedef ::testing::Types<
    force::Force3D,
    force::Force3F
> Types;


TYPED_TEST_CASE(Force3Test, Types);

TYPED_TEST(Force3Test, testForce3)
{
  typedef typename TestFixture::Force3 Force3;

  // default constructor
  ASSERT_EQ(this->forceDefault.x(), this->vecZero.x()) << "Default constructor needs to initialize x-component to zero!";
  ASSERT_EQ(this->forceDefault.y(), this->vecZero.y()) << "Default constructor needs to initialize y-component to zero!";
  ASSERT_EQ(this->forceDefault.z(), this->vecZero.z()) << "Default constructor needs to initialize z-component to zero!";

  // constructor with three values (x,y,z)
  ASSERT_EQ(this->forceFromThreeValues.x(), this->vec1.x()) << "Three-Value Constructor needs to first initialize x-component!";
  ASSERT_EQ(this->forceFromThreeValues.y(), this->vec1.y()) << "Three-Value Constructor needs to second initialize y-component!";
  ASSERT_EQ(this->forceFromThreeValues.z(), this->vec1.z()) << "Three-Value Constructor needs to third initialize z-component!";

  // constructor with Eigen force
  ASSERT_EQ(this->forceFromEigen.x(), this->vec1.x()) << "Base Constructor needs to first initialize x-component!";
  ASSERT_EQ(this->forceFromEigen.y(), this->vec1.y()) << "Base Constructor needs to second initialize y-component!";
  ASSERT_EQ(this->forceFromEigen.z(), this->vec1.z()) << "Base Constructor needs to third initialize z-component!";

  // constructor with Force3
  ASSERT_EQ(this->forceFromForce.x(), this->vec1.x());
  ASSERT_EQ(this->forceFromForce.y(), this->vec1.y());
  ASSERT_EQ(this->forceFromForce.z(), this->vec1.z());

  // toImplementation
  ASSERT_EQ(this->forceFromThreeValues.toImplementation()(0,0), this->vec1.x()) << "X-component needs to correspond to the matrix entry (0,0)!";
  ASSERT_EQ(this->forceFromThreeValues.toImplementation()(1,0), this->vec1.y()) << "Y-component needs to correspond to the matrix entry (1,0)!";
  ASSERT_EQ(this->forceFromThreeValues.toImplementation()(2,0), this->vec1.z()) << "Z-component needs to correspond to the matrix entry (2,0)!";

  // addition
  Force3 vectorAdd = this->forceFromEigen+this->force2FromEigen;
  ASSERT_EQ(vectorAdd.x(), this->vecAdd.x());
  ASSERT_EQ(vectorAdd.y(), this->vecAdd.y());
  ASSERT_EQ(vectorAdd.z(), this->vecAdd.z());

  // addition and assignment
  Force3 vectorAddandAssign(this->forceFromEigen);
  vectorAddandAssign += this->force2FromEigen;
  ASSERT_EQ(vectorAddandAssign.x(), this->vecAdd.x());
  ASSERT_EQ(vectorAddandAssign.y(), this->vecAdd.y());
  ASSERT_EQ(vectorAddandAssign.z(), this->vecAdd.z());

  // subtract
  Force3 vectorSubtract = this->forceFromEigen-this->force2FromEigen;
  ASSERT_EQ(vectorSubtract.x(), this->vecSubtract.x());
  ASSERT_EQ(vectorSubtract.y(), this->vecSubtract.y());
  ASSERT_EQ(vectorSubtract.z(), this->vecSubtract.z());

  // subtract and assignment
  Force3 vectorSubtractandAssign(this->forceFromEigen);
  vectorSubtractandAssign -= this->force2FromEigen;
  ASSERT_EQ(vectorSubtractandAssign.x(), this->vecSubtract.x());
  ASSERT_EQ(vectorSubtractandAssign.y(), this->vecSubtract.y());
  ASSERT_EQ(vectorSubtractandAssign.z(), this->vecSubtract.z());

  // setZero
  Force3 vec2backup(this->vec2);
  this->vec2.setZero();
  ASSERT_EQ(this->vec2.x(), this->vecZero.x());
  ASSERT_EQ(this->vec2.y(), this->vecZero.y());
  ASSERT_EQ(this->vec2.z(), this->vecZero.z());

  // assign 1
  this->vec2(0) = vec2backup(0);
  ASSERT_EQ(this->vec2.x(), vec2backup.x());

  // assign 2
  this->vec2 << vec2backup(0),vec2backup(1),vec2backup(2);
  ASSERT_EQ(this->vec2.x(), vec2backup.x());
  ASSERT_EQ(this->vec2.y(), vec2backup.y());
  ASSERT_EQ(this->vec2.z(), vec2backup.z());
}









template <typename ForceImplementation>
struct ForceTest: public ::testing::Test {
  typedef ForceImplementation Force;
  typedef typename Force::Scalar Scalar;
//  const int length = Force::Dimension;
  typedef typename Force::Implementation EigenVector;

  Scalar tol;
  EigenVector vecZero, vec1, vec2, vecAdd, vecSubtract;

  Force forceDefault;
  Force vectorFromMultipleValues;
  Force forceFromEigen;
  Force force2FromEigen;
  Force forceFromForce;

  ForceTest() : tol(1e-6)
  {
    vecZero = EigenVector::Zero();
    vec1 << 10,20,30,40,50;
    vec2 << 1,2,3,4,5;
    vecAdd << 11,22,33,44,55;
    vecSubtract << 9,18,27,36,45;
    vectorFromMultipleValues << vec1(0),vec1(1),vec1(2),vec1(3),vec1(4);
    forceFromEigen(vec1);
    force2FromEigen(vec2);
    forceFromForce = forceFromEigen;
  }
};



typedef ::testing::Types<
    force::Force<double, 5>,
    force::Force<float, 5>
> Types5D;


TYPED_TEST_CASE(ForceTest, Types5D);

TYPED_TEST(ForceTest, testForce)
{
  typedef typename TestFixture::Force Force;

  // default constructor
  ASSERT_EQ(this->forceDefault(0), this->vecZero(0)) << "Default constructor needs to initialize component 1 to zero!";
  ASSERT_EQ(this->forceDefault(1), this->vecZero(1)) << "Default constructor needs to initialize component 2 to zero!";
  ASSERT_EQ(this->forceDefault(2), this->vecZero(2)) << "Default constructor needs to initialize component 3 to zero!";
  ASSERT_EQ(this->forceDefault(3), this->vecZero(3)) << "Default constructor needs to initialize component 4 to zero!";
  ASSERT_EQ(this->forceDefault(4), this->vecZero(4)) << "Default constructor needs to initialize component 5 to zero!";

  // constructor with multiple values
  ASSERT_EQ(this->vectorFromMultipleValues(0), this->vec1(0)) << "Multi-Value Constructor needs to initialize component 1!";
  ASSERT_EQ(this->vectorFromMultipleValues(1), this->vec1(1)) << "Multi-Value Constructor needs to initialize component 2!";
  ASSERT_EQ(this->vectorFromMultipleValues(2), this->vec1(2)) << "Multi-Value Constructor needs to initialize component 3!";
  ASSERT_EQ(this->vectorFromMultipleValues(3), this->vec1(3)) << "Multi-Value Constructor needs to initialize component 4!";
  ASSERT_EQ(this->vectorFromMultipleValues(4), this->vec1(4)) << "Multi-Value Constructor needs to initialize component 5!";

  // constructor with Eigen force
  ASSERT_EQ(this->forceFromEigen(0), this->vec1(0)) << "Base Constructor needs to initialize component 1!";
  ASSERT_EQ(this->forceFromEigen(1), this->vec1(1)) << "Base Constructor needs to initialize component 2!";
  ASSERT_EQ(this->forceFromEigen(2), this->vec1(2)) << "Base Constructor needs to initialize component 3!";
  ASSERT_EQ(this->forceFromEigen(3), this->vec1(3)) << "Base Constructor needs to initialize component 4!";
  ASSERT_EQ(this->forceFromEigen(4), this->vec1(4)) << "Base Constructor needs to initialize component 5!";

  // constructor with Force
  ASSERT_EQ(this->forceFromForce(0), this->vec1(0));
  ASSERT_EQ(this->forceFromForce(1), this->vec1(1));
  ASSERT_EQ(this->forceFromForce(2), this->vec1(2));
  ASSERT_EQ(this->forceFromForce(3), this->vec1(3));
  ASSERT_EQ(this->forceFromForce(4), this->vec1(4));

  // toImplementation
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(0,0), this->vec1(0)) << "Component 1 needs to correspond to the matrix entry (0,0)!";
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(1,0), this->vec1(1)) << "Component 2 needs to correspond to the matrix entry (1,0)!";
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(2,0), this->vec1(2)) << "Component 3 needs to correspond to the matrix entry (2,0)!";
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(3,0), this->vec1(3)) << "Component 4 needs to correspond to the matrix entry (1,0)!";
  ASSERT_EQ(this->vectorFromMultipleValues.toImplementation()(4,0), this->vec1(4)) << "Component 5 needs to correspond to the matrix entry (2,0)!";

  // addition
  Force vectorAdd = this->forceFromEigen+this->force2FromEigen;
  ASSERT_EQ(vectorAdd(0), this->vecAdd(0));
  ASSERT_EQ(vectorAdd(1), this->vecAdd(1));
  ASSERT_EQ(vectorAdd(2), this->vecAdd(2));
  ASSERT_EQ(vectorAdd(3), this->vecAdd(3));
  ASSERT_EQ(vectorAdd(4), this->vecAdd(4));

  // addition and assignment
  Force vectorAddandAssign(this->forceFromEigen);
  vectorAddandAssign += this->force2FromEigen;
  ASSERT_EQ(vectorAddandAssign(0), this->vecAdd(0));
  ASSERT_EQ(vectorAddandAssign(1), this->vecAdd(1));
  ASSERT_EQ(vectorAddandAssign(2), this->vecAdd(2));
  ASSERT_EQ(vectorAddandAssign(3), this->vecAdd(3));
  ASSERT_EQ(vectorAddandAssign(4), this->vecAdd(4));

  // subtract
  Force vectorSubtract = this->forceFromEigen-this->force2FromEigen;
  ASSERT_EQ(vectorSubtract(0), this->vecSubtract(0));
  ASSERT_EQ(vectorSubtract(1), this->vecSubtract(1));
  ASSERT_EQ(vectorSubtract(2), this->vecSubtract(2));
  ASSERT_EQ(vectorSubtract(3), this->vecSubtract(3));
  ASSERT_EQ(vectorSubtract(4), this->vecSubtract(4));

  // subtract and assignment
  Force vectorSubtractandAssign(this->forceFromEigen);
  vectorSubtractandAssign -= this->force2FromEigen;
  ASSERT_EQ(vectorSubtractandAssign(0), this->vecSubtract(0));
  ASSERT_EQ(vectorSubtractandAssign(1), this->vecSubtract(1));
  ASSERT_EQ(vectorSubtractandAssign(2), this->vecSubtract(2));
  ASSERT_EQ(vectorSubtractandAssign(3), this->vecSubtract(3));
  ASSERT_EQ(vectorSubtractandAssign(4), this->vecSubtract(4));

  // setZero
  Force vec2backup(this->vec2);
  this->vec2.setZero();
  ASSERT_EQ(this->vec2(0), this->vecZero(0));
  ASSERT_EQ(this->vec2(1), this->vecZero(1));
  ASSERT_EQ(this->vec2(2), this->vecZero(2));
  ASSERT_EQ(this->vec2(3), this->vecZero(3));
  ASSERT_EQ(this->vec2(4), this->vecZero(4));

  // assign 1
  this->vec2(0) = vec2backup(0);
  ASSERT_EQ(this->vec2(0), vec2backup(0));

  // assign 2
  this->vec2 << vec2backup(0),vec2backup(1),vec2backup(2),vec2backup(3),vec2backup(4);
  ASSERT_EQ(this->vec2(0), vec2backup(0));
  ASSERT_EQ(this->vec2(1), vec2backup(1));
  ASSERT_EQ(this->vec2(2), vec2backup(2));
  ASSERT_EQ(this->vec2(3), vec2backup(3));
  ASSERT_EQ(this->vec2(4), vec2backup(4));
}

