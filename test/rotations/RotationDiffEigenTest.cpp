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

#include "kindr/rotations/RotationDiffEigen.hpp"
#include "kindr/common/gtest_eigen.hpp"

namespace rot = kindr::rotations::eigen_impl;


TEST(RotationDiffTest, DISABLED_testDevelopment)
{
//  rot::RotationQuaternionPD rquat;
//
//  rot::RotationQuaternionDiffPD rquatdiff(1,2,3,4);
//  rot::LocalAngularVelocityPD angularVelocity(rquat, rquatdiff);
//
//
//  std::cout << "rquatdiff:" << rquatdiff << std::endl;
//  std::cout << "angularVelocity:" << angularVelocity << std::endl;
//  std::cout << "angularVelocity2" << rquatdiff.cast<rot::LocalAngularVelocityPD>(rquat) << std::endl;
//
//
//  rot::EulerAnglesXyzPD eulerAnglesXyz(0.1, 0.2, 0.3);
//  rot::LocalAngularVelocityPD angularVelocity2(0.2, 0.2, 0.2);
//  rot::EulerAnglesXyzDiffPD eulerAnglesXyzDiff2(eulerAnglesXyz,angularVelocity2);
//
//  std::cout << "eulerDiff: " << eulerAnglesXyzDiff2 << std::endl;

//  rot::AngleAxisAD aa;
// rot::AngleAxisDiffAD aaDiff(0.2, 0.1, 0.2, 0.4);
// std::cout << "angle axis: " << aaDiff << std::endl;
// rot::LocalAngularVelocityAD avelA6(aa, aaDiff);
//
//  rot::RotationMatrixDiffAD rmatADiff;
//  rot::RotationMatrixAD rmatA;
////  rot::LocalAngularVelocityAD avelA(rmatA, rmatADiff);
////  std::cout << "avelA: " << avelA << std::endl;
//
//  rot::RotationMatrixDiffPD rmatPDiff;
//  rot::RotationMatrixPD rmatP;
//  rot::LocalAngularVelocityAD avelA2(rmatA, rmatADiff);
//  rot::LocalAngularVelocityAD avelA3(rmatP, rmatPDiff);
//  std::cout << "avelA2: " << avelA2 << std::endl;
//
//  rot::RotationVectorDiffAD rvADiff;
//  rot::RotationVectorAD rvA;
//  rot::LocalAngularVelocityAD avelAz(rvA, rvADiff);
////  std::cout << rvA << std::endl;

  rot::LocalAngularVelocityAD avA(0.9, 0.5, 0.8);

  rot::AngleAxisAD aaA(0.2, 0.0, 1.0, 0.0);
  rot::AngleAxisDiffAD aaDiffA(aaA, avA);
  rot::LocalAngularVelocityAD avA2(aaA, aaDiffA);

  std::cout << "AngleAxisDiffAD: avA | avA2: "<<  avA << " | "  << avA2 << std::endl;


  // Finite difference method for checking derivatives
  rot::AngleAxisAD aaAnext;
  rot::AngleAxisDiffAD aaDiffA2;
  double dt = 0.00000001;
  aaAnext = aaA.boxPlus(dt*avA.toImplementation());
  double dtheta = (aaAnext.angle()-aaA.angle())/dt;
  rot::AngleAxisDiffAD::Vector3 dn = (aaAnext.axis()-aaA.axis())/dt;
  ASSERT_NEAR(aaDiffA.angle(),dtheta,1e-6);
  ASSERT_NEAR(aaDiffA.axis()(0),dn(0),1e-6);
  ASSERT_NEAR(aaDiffA.axis()(1),dn(1),1e-6);
  ASSERT_NEAR(aaDiffA.axis()(2),dn(2),1e-6);



  rot::RotationQuaternionAD rqA(aaA);
  rot::RotationQuaternionDiffAD rqDiffA(rqA, avA);

  rot::LocalAngularVelocityAD avA3(rqA, rqDiffA);

  std::cout << "RotationQuaternionDiffAD: avA | avA3: "<<  avA << " | "  << avA3 << std::endl;

  rot::EulerAnglesZyxAD zyxA(2,0,0.2);
  rot::EulerAnglesZyxDiffAD zyxDiffA(zyxA, avA);
  rot::LocalAngularVelocityAD avA8(zyxA, zyxDiffA);

  std::cout << "EulerAnglesZyxDiffAD: avA | avA8: "<<  avA << " | "  << avA8 << std::endl;

  rot::EulerAnglesXyzAD xyzA(2,0,0.2);
  rot::EulerAnglesXyzDiffAD xyzDiffA(xyzA, avA);
  rot::LocalAngularVelocityAD avA9(xyzA, xyzDiffA);

  std::cout << "EulerAnglesXyzDiffAD: avA | avA9: "<<  avA << " | "  << avA9 << std::endl;


  rot::RotationVectorAD rvA(rqA);
  rot::RotationVectorDiffAD rvDiffA(rvA, avA);
  rot::LocalAngularVelocityAD avA10(rvA, rvDiffA);

  std::cout << "RotationVectorDiffAD: avA | avA9: "<<  avA << " | "  << avA10 << std::endl;


//  rot::RotationQuaternionDiffAD rqDiffA2(rvA, rvDiffA);
//  std::cout << "RotationQuaternionDiffAD: rqDiffA | rqDiffA2: "<<  rqDiffA << " | "  << rqDiffA2 << std::endl;
//  rot::RotationQuaternionDiffAD rqDiffA3(rqA, rvDiffA);
//  std::cout << "RotationQuaternionDiffAD: rqDiffA | rqDiffA3: "<<  rqDiffA << " | "  << rqDiffA3 << std::endl;

  rot::RotationMatrixAD rmA(rqA);
  rot::RotationMatrixDiffAD rmDiffA(rmA, avA);
  rot::LocalAngularVelocityAD avA11(rmA, rmDiffA);
  std::cout << "RotationMatrixDiffAD: avA | avA11: "<<  avA << " | "  << avA11 << std::endl;

//  rot::RotationMatrixPD rmP(rqA.getPassive());
//  rot::RotationMatrixDiffPD rmDiffA2(rmP, avA);
//  rot::LocalAngularVelocityPD avP12(rmP, rmDiffA2);
//  std::cout << "RotationMatrixDiffPD: avA | avA12: "<<  avA << " | "  << avP12 << std::endl;

  Eigen::Vector3d vector(2,0,0);
  rot::AngleAxisAD aaTest;
  aaTest.setExponentialMap(vector);
  std::cout << "aaTest exp: " << aaTest << std::endl;

  std::cout << "aaTest log: " << aaTest.getLogarithmicMap() << std::endl;

  Eigen::Vector3d vector2 = aaTest.boxMinus(rqA);
  rot::AngleAxisAD rot = aaTest.boxPlus(vector2);

//  rot::AngleAxisDiffAD test1(0.3, 1.0, 2.0, 3.0);
//  rot::AngleAxisDiffAD test2 = test1;
//  std::cout << "test2=: " << test2;

}

