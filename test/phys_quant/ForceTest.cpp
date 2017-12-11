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

namespace vectors = kindr;

template <typename ForceImplementation>
struct ForceTorqueTest: public ::testing::Test {
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

  ForceTorqueTest() : tol(1e-6)
  {

  }
};


typedef ::testing::Types<
    vectors::VectorTypeless3D
> Types3;


TYPED_TEST_CASE(ForceTorqueTest, Types3);

TYPED_TEST(ForceTorqueTest, testForce)
{
  typedef vectors::VectorTypeless3D Vector;
  typedef vectors::Force3D Force;
  typedef vectors::Torque3D Torque;

  Vector v(1,2,3);
  Force f1(v);
  Torque t1(v);

//  Force f2 = t1 + f1;
  std::cout << f1 << std::endl;
  f1 = Force::Zero();
  std::cout << f1 << std::endl;
}

