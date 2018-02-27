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

#include "kindr/phys_quant/PhysicalQuantities.hpp"
#include "kindr/common/gtest_eigen.hpp"



namespace pos = kindr;

template <typename PositionImplementation>
struct Position3Test: public ::testing::Test {
  typedef PositionImplementation Pos3;
  typedef typename Pos3::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

  Scalar tol;
  Scalar factor;
  Scalar divisor;
  Vector3 vecZero, vec1, vec2, vecAdd, vecSubtract, vecMult, vecDiv;

  Pos3 posDefault;
  Pos3 posFromThreeValues;
  Pos3 posFromEigen;
  Pos3 pos2FromEigen;
  Pos3 posFromPos;

  Position3Test() : tol(1e-6),
      factor(2),
      divisor(2),
      vecZero(Vector3::Zero()),
      vec1(10,20,30),
      vec2(1,2,3),
      vecAdd(11,22,33),
      vecSubtract(9,18,27),
      vecMult(20,40,60),
      vecDiv(5,10,15),
      posFromThreeValues(vec1.x(), vec1.y(), vec1.z()),
      posFromEigen(vec1),
      pos2FromEigen(vec2),
      posFromPos(posFromEigen)
  {}
};


typedef ::testing::Types<
    pos::Position3D,
    pos::Position3F
> Types;


TYPED_TEST_CASE(Position3Test, Types);

TYPED_TEST(Position3Test, testPosition3)
{
  typedef typename TestFixture::Pos3 Pos3;

  // default constructor
  ASSERT_EQ(this->posDefault.x(), this->vecZero.x()) << "Default constructor needs to initialize x-component to zero!";
  ASSERT_EQ(this->posDefault.y(), this->vecZero.y()) << "Default constructor needs to initialize y-component to zero!";
  ASSERT_EQ(this->posDefault.z(), this->vecZero.z()) << "Default constructor needs to initialize z-component to zero!";

  // constructor with three values (x,y,z)
  ASSERT_EQ(this->posFromThreeValues.x(), this->vec1.x()) << "Three-Value Constructor needs to first initialize x-component!";
  ASSERT_EQ(this->posFromThreeValues.y(), this->vec1.y()) << "Three-Value Constructor needs to second initialize y-component!";
  ASSERT_EQ(this->posFromThreeValues.z(), this->vec1.z()) << "Three-Value Constructor needs to third initialize z-component!";

  // constructor with Eigen vector
  ASSERT_EQ(this->posFromEigen.x(), this->vec1.x()) << "Base Constructor needs to first initialize x-component!";
  ASSERT_EQ(this->posFromEigen.y(), this->vec1.y()) << "Base Constructor needs to second initialize y-component!";
  ASSERT_EQ(this->posFromEigen.z(), this->vec1.z()) << "Base Constructor needs to third initialize z-component!";

  // constructor with Position3
  ASSERT_EQ(this->posFromPos.x(), this->vec1.x());
  ASSERT_EQ(this->posFromPos.y(), this->vec1.y());
  ASSERT_EQ(this->posFromPos.z(), this->vec1.z());

  // toImplementation
  ASSERT_EQ(this->posFromThreeValues.toImplementation()(0,0), this->vec1.x()) << "X-component needs to correspond to the matrix entry (0,0)!";
  ASSERT_EQ(this->posFromThreeValues.toImplementation()(1,0), this->vec1.y()) << "Y-component needs to correspond to the matrix entry (1,0)!";
  ASSERT_EQ(this->posFromThreeValues.toImplementation()(2,0), this->vec1.z()) << "Z-component needs to correspond to the matrix entry (2,0)!";

  // addition
  Pos3 posAdd = this->posFromEigen+this->pos2FromEigen;
  ASSERT_EQ(posAdd.x(), this->vecAdd.x());
  ASSERT_EQ(posAdd.y(), this->vecAdd.y());
  ASSERT_EQ(posAdd.z(), this->vecAdd.z());

  // addition and assignment
  Pos3 posAddandAssign(this->posFromEigen);
  posAddandAssign += this->pos2FromEigen;
  ASSERT_EQ(posAddandAssign.x(), this->vecAdd.x());
  ASSERT_EQ(posAddandAssign.y(), this->vecAdd.y());
  ASSERT_EQ(posAddandAssign.z(), this->vecAdd.z());

  // subtraction
  Pos3 posSubtract = this->posFromEigen-this->pos2FromEigen;
  ASSERT_EQ(posSubtract.x(), this->vecSubtract.x());
  ASSERT_EQ(posSubtract.y(), this->vecSubtract.y());
  ASSERT_EQ(posSubtract.z(), this->vecSubtract.z());

  // subtraction and assignment
  Pos3 posSubtractandAssign(this->posFromEigen);
  posSubtractandAssign -= this->pos2FromEigen;
  ASSERT_EQ(posSubtractandAssign.x(), this->vecSubtract.x());
  ASSERT_EQ(posSubtractandAssign.y(), this->vecSubtract.y());
  ASSERT_EQ(posSubtractandAssign.z(), this->vecSubtract.z());

  // multiplication a)
  Pos3 posMultA = this->posFromEigen * this->factor;
  ASSERT_EQ(posMultA.x(), this->vecMult.x());
  ASSERT_EQ(posMultA.y(), this->vecMult.y());
  ASSERT_EQ(posMultA.z(), this->vecMult.z());

  // multiplication b)
  Pos3 posMultB = this->factor * this->posFromEigen;
  ASSERT_EQ(posMultB.x(), this->vecMult.x());
  ASSERT_EQ(posMultB.y(), this->vecMult.y());
  ASSERT_EQ(posMultB.z(), this->vecMult.z());

  // multiplication and assignment
  Pos3 posMultAndAssign(this->posFromEigen);
  posMultAndAssign *= this->factor;
  ASSERT_EQ(posMultAndAssign.x(), this->vecMult.x());
  ASSERT_EQ(posMultAndAssign.y(), this->vecMult.y());
  ASSERT_EQ(posMultAndAssign.z(), this->vecMult.z());

  // division
  Pos3 posDiv = this->posFromEigen / this->divisor;
  ASSERT_EQ(posDiv.x(), this->vecDiv.x());
  ASSERT_EQ(posDiv.y(), this->vecDiv.y());
  ASSERT_EQ(posDiv.z(), this->vecDiv.z());

  // division and assignment
  Pos3 posDivAndAssign(this->posFromEigen);
  posDivAndAssign /= this->divisor;
  ASSERT_EQ(posDivAndAssign.x(), this->vecDiv.x());
  ASSERT_EQ(posDivAndAssign.y(), this->vecDiv.y());
  ASSERT_EQ(posDivAndAssign.z(), this->vecDiv.z());
}

