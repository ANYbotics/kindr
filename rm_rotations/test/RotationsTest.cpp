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
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include <gtest/gtest.h>

#include "rm/common/common.hpp"
#include "rm/common/gtest_eigen.hpp"
#include "rm/rotations/RotationEigen.hpp"
#include "rm/quaternions/QuaternionEigen.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

//#include <sm/eigen/gtest.hpp>
//#include <sm/random.hpp>
//#include <sm/timing/Timer.hpp>

#include <limits>
#include <iostream>
#include <random>



TEST (RotationsTest, DISABLED_testEigenEulerAngleRange ) {
  using namespace Eigen;
  using namespace rm::rotations;

//  std::cout << getRpyFromQuaternion(getQuaternionFromRpy(Vector3d(M_PI,0,0))).transpose() << std::endl;
//  std::cout << getRpyFromQuaternion(getQuaternionFromRpy(Vector3d(-M_PI,0,0))).transpose() << std::endl;
//  std::cout << getRpyFromQuaternion(getQuaternionFromRpy(Vector3d(0,M_PI/2,0))).transpose() << std::endl;
//  std::cout << getRpyFromQuaternion(getQuaternionFromRpy(Vector3d(0,-M_PI/2,0))).transpose() << std::endl;
//  std::cout << getRpyFromQuaternion(getQuaternionFromRpy(Vector3d(0,0,M_PI))).transpose() << std::endl;
//  std::cout << getRpyFromQuaternion(getQuaternionFromRpy(Vector3d(0,0,-M_PI))).transpose() << std::endl;
//
//  std::cout << std::endl;
//
//  double eps = 0.001;
//
//  std::cout << getRpyFromAngleAxis(AngleAxisd(M_PI,Vector3d(1,0,0))).transpose() << std::endl;
//  std::cout << getRpyFromAngleAxis(AngleAxisd(-M_PI,Vector3d(1,0,0))).transpose() << std::endl;
//  std::cout << getRpyFromAngleAxis(AngleAxisd(M_PI,Vector3d(0,1,0))).transpose() << std::endl;
//  std::cout << getRpyFromAngleAxis(AngleAxisd(-M_PI,Vector3d(0,1,0))).transpose() << std::endl;
//  std::cout << getRpyFromAngleAxis(AngleAxisd(M_PI,Vector3d(0,0,1))).transpose() << std::endl;
//  std::cout << getRpyFromAngleAxis(AngleAxisd(-M_PI,Vector3d(0,0,1))).transpose() << std::endl;
}

//TEST (RotationsTest, DISABLED_testWrapAngle ) {
//  using namespace rm::common;
//  sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));
//
//  for(int i=0; i<1e6; i++)
//  {
//    double angle = sm::random::randLU(-100,100);
//
//    EXPECT_EQ(wrapPosNegPI(angle), wrapAngle(angle,-M_PI,M_PI));
//  }
//}
//
//TEST (RotationsTest, DISABLED_testQuaternionMultiplication ) {
//  using namespace Eigen;
//  using namespace rm::rotations;
//  sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));
//
//  Vector3d n1 = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
//  n1.normalize();
//  AngleAxisd aa_AI = AngleAxisd(sm::random::randLU(-M_PI,M_PI),n1);
//  Quaterniond p_AI = Quaterniond(aa_AI);
//
//  Vector3d n2 = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
//  n2.normalize();
//  AngleAxisd aa_BA = AngleAxisd(sm::random::randLU(-M_PI,M_PI),n2);
//  Quaterniond p_BA = Quaterniond(aa_BA);
//
//  Quaterniond p_BI1 = p_BA*p_AI;
//
//  double w = p_BA.w()*p_AI.w() - p_BA.x()*p_AI.x() - p_BA.y()*p_AI.y() - p_BA.z()*p_AI.z();
//  double x = p_BA.w()*p_AI.x() + p_AI.w()*p_BA.x() + p_BA.y()*p_AI.z() - p_BA.z()*p_AI.y();
//  double y = p_BA.w()*p_AI.y() + p_AI.w()*p_BA.y() + p_BA.z()*p_AI.x() - p_BA.x()*p_AI.z();
//  double z = p_BA.w()*p_AI.z() + p_AI.w()*p_BA.z() + p_BA.x()*p_AI.y() - p_BA.y()*p_AI.x();
//  Quaterniond p_BI2 = Quaterniond(w,x,y,z);
//
////  ASSERT_MATRIX_NEAR(quaternionToVector(p_BI1), quaternionToVector(p_BI2), 1e-6, "p");
//
//  Vector3d I_r = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
//
//  Vector3d B_r1 = (p_BA * p_AI) * I_r;
//  Vector3d B_r2 = p_BA * (p_AI * I_r);
//
////  ASSERT_MATRIX_NEAR(B_r1, B_r2, 1e-6, "p");
//}
//
//TEST (RotationsTest, DISABLED_testRotationFunctions ) {
//  using namespace Eigen;
//  using namespace rm::rotations;
//
//  // random seed
//    sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));
//
////    Quaterniond p_IB = Quaterniond(cos(M_PI/12),0,0,sin(M_PI/12));
////     std::cout << getRpyFromQuaternion(p_IB).transpose() << std::endl;
////     std::cout << getYprFromQuaternion(p_IB).transpose() << std::endl;
////
////     Vector3d rpy_IB2 = Vector3d(0,0,2);
////     Quaterniond test = getQuaternionFromRpy(rpy_IB2);
////     std::cout << test.w() << test.x() << test.y() << test.z() << std::endl;
////
////     Matrix3d A_BI = Matrix3d::Zero();
////     A_BI << cos(M_PI/6), sin(M_PI/6), 0, -sin(M_PI/6), cos(M_PI/6), 0, 0, 0, 1;
////     std::cout << getRpyFromTransformationMatrix(A_BI) << std::endl;
////     std::cout << getAngleAxisFromTransformationMatrix(A_BI).angle() << std::endl;
////     std::cout << getAngleAxisFromTransformationMatrix(A_BI).axis().transpose() << std::endl;
////
////     Vector3d rpy_IB2 = Vector3d(0,0,1);
////     std::cout << getTransformationMatrixFromRpy(rpy_IB2) << std::endl;
////
////     std::cout << getAngleAxisFromRpy(rpy_IB2).axis() << std::endl << getAngleAxisFromRpy(rpy_IB2).angle() << std::endl;
//
////    std::cout << getTransformationMatrixFromAngleAxis(AngleAxisd(-M_PI/6,Vector3d(0,0,1))) << std::endl;
////    std::cout << getTransformationMatrixFromQuaternion(Quaterniond(AngleAxisd(-M_PI/6,Vector3d(0,0,1)))) << std::endl;
//
////    Vector3d I_r = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
////    Vector3d I_r_p =
////    std::cout << getTransformationMatrixFromQuaternion(Quaterniond(AngleAxisd(M_PI/6,Vector3d(0,0,1))))*I_r << std::endl;
//
////     std::cout << getTransformationMatrixFromQuaternion(Quaterniond(cos(M_PI/6),0,0,sin(M_PI/6))) << std::endl;
//
////    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).w() << std::endl;
////    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).x() << std::endl;
////    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).y() << std::endl;
////    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).z() << std::endl;
//
////    std::cout << getRpyFromQuaternion(Quaterniond(cos(M_PI/2),0,sin(M_PI/2),0)) << std::endl;
//
//
//    for(int i=0; i<1e5; i++)
//    {
//      Vector3d n0 = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
//      n0.normalize();
//      AngleAxisd aa_IB0 = AngleAxisd(sm::random::randLU(-M_PI,M_PI),n0);
//
////      Vector3d rpy_IB = Vector3d(sm::random::randLU(-M_PI,M_PI),sm::random::randLU(-M_PI,M_PI),sm::random::randLU(-M_PI,M_PI));
////      Vector3d rpy_IB = Vector3d(sm::random::randLU(-M_PI,M_PI),M_PI/2,sm::random::randLU(-M_PI,M_PI));
////      AngleAxisd aa_IB0 = getAngleAxisFromRpy(rpy_IB);
//
//      Quaterniond p_IB0 = Quaterniond(aa_IB0);
//      Matrix3d A_BI0 = p_IB0.toRotationMatrix();
////      Vector3d rpy_IB0 = getRpyFromTransformationMatrix(A_BI0);
////      Vector3d ypr_IB0 = getYprFromTransformationMatrix(A_BI0);
//
////      std::cout << std::endl << std::endl;
////      std::cout << "angle = " << aa_IB0.angle() << ", axis = " << aa_IB0.axis().transpose() << std::endl;
////      std::cout << "quaternion = " << quaternionToVector(p_IB0).transpose() << std::endl;
////      std::cout << "trafo = " << std::endl << A_BI0 << std::endl;
////      std::cout << "rpy = " << rpy_IB0.transpose() << std::endl;
////      std::cout << "ypr = " << ypr_IB0.transpose() << std::endl;
//
////      std::cout << std::endl << std::endl;
////      std::cout << "p0 = " << quaternionToVector(p_IB0).transpose() << std::endl;
////      std::cout << "p1 = " << quaternionToVector(getQuaternionFromTransformationMatrix(getTransformationMatrixFromRpy(rpy_IB0))).transpose() << std::endl;
////      std::cout << "p2 = " << quaternionToVector(getQuaternionFromRpy(rpy_IB0)).transpose() << std::endl;
//
////      ASSERT_MATRIX_NEAR(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromQuaternion(p_IB0).axis()*getAngleAxisFromQuaternion(p_IB0).angle(), 3e-4, "aa1"); // ok
////      ASSERT_MATRIX_NEAR(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromTransformationMatrix(A_BI0).axis()*getAngleAxisFromTransformationMatrix(A_BI0).angle(), 3e-4, "aa2"); // ok
////      ASSERT_MATRIX_NEAR(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromRpy(rpy_IB0).axis()*getAngleAxisFromRpy(rpy_IB0).angle(), 3e-4, "aa3"); // ok
////      ASSERT_MATRIX_NEAR(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromYpr(ypr_IB0).axis()*getAngleAxisFromYpr(ypr_IB0).angle(), 3e-4, "aa4"); // ok
////
////      ASSERT_MATRIX_NEAR(quaternionToVector(p_IB0), quaternionToVector(getQuaternionFromAngleAxis(aa_IB0)), 1e-6, "quat1"); // ok
////      ASSERT_MATRIX_NEAR(quaternionToVector(p_IB0), quaternionToVector(getQuaternionFromTransformationMatrix(A_BI0)), 1e-6, "quat2"); // ok
////      ASSERT_MATRIX_NEAR(quaternionToVector(p_IB0), quaternionToVector(getQuaternionFromRpy(rpy_IB0)), 1e-6, "quat3"); // ok
////      ASSERT_MATRIX_NEAR(quaternionToVector(p_IB0), quaternionToVector(getQuaternionFromYpr(ypr_IB0)), 1e-6, "quat4"); // ok
////
////      ASSERT_MATRIX_NEAR(A_BI0, getTransformationMatrixFromAngleAxis(aa_IB0), 1e-6, "trafo1"); // ok
////      ASSERT_MATRIX_NEAR(A_BI0, getTransformationMatrixFromQuaternion(p_IB0), 1e-6, "trafo2"); // ok
////      ASSERT_MATRIX_NEAR(A_BI0, getTransformationMatrixFromRpy(rpy_IB0), 1e-6, "trafo3"); // ok
////      ASSERT_MATRIX_NEAR(A_BI0, getTransformationMatrixFromYpr(ypr_IB0), 1e-6, "trafo4"); // ok
////
////      ASSERT_MATRIX_NEAR(rpy_IB0, getRpyFromAngleAxis(aa_IB0), 1e-6, "rpy1"); // ok
////      ASSERT_MATRIX_NEAR(rpy_IB0, getRpyFromQuaternion(p_IB0), 1e-6, "rpy2"); // ok
////      ASSERT_MATRIX_NEAR(rpy_IB0, getRpyFromTransformationMatrix(A_BI0), 1e-6, "rpy3"); // ok
////      ASSERT_MATRIX_NEAR(rpy_IB0, getRpyFromYpr(ypr_IB0), 1e-6, "rpy4"); // ok
////
////      ASSERT_MATRIX_NEAR(ypr_IB0, getYprFromAngleAxis(aa_IB0), 1e-6, "ypr1"); // ok
////      ASSERT_MATRIX_NEAR(ypr_IB0, getYprFromQuaternion(p_IB0), 1e-6, "ypr2"); // ok
////      ASSERT_MATRIX_NEAR(ypr_IB0, getYprFromTransformationMatrix(A_BI0), 1e-6, "ypr3"); // ok
////      ASSERT_MATRIX_NEAR(ypr_IB0, getYprFromRpy(rpy_IB0), 1e-6, "ypr4"); // ok
//    }
//}


namespace rot = rm::rotations::eigen_implementation;
namespace quat = rm::quaternions::eigen_implementation;

template <typename RotationImplementation>
struct RotationsTest : public ::testing::Test  {
  typedef typename RotationImplementation::Scalar Scalar;
  static constexpr rm::rotations::RotationUsage Usage = RotationImplementation::Usage;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  Scalar tol;
  Vector3 X, Y, Z, Vgeneric;

//  typedef typename GenericScalar_::Scalar PrimScalar;
//  typedef GenericScalarExpression<PrimScalar> TestGenericScalarExpression;
//  typedef GenericScalar_ TestGenericScalar;

//  static PrimScalar getRandScalar() {
//    return PrimScalar(sm::random::rand() * 10.0);
//  }
  RotationImplementation stdconstr;
  RotationImplementation identity = RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(1, 0, 0, 0).cast<Scalar>()));
  RotationImplementation halfX =    RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(0, 1, 0, 0).cast<Scalar>()));
  RotationImplementation halfY =    RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(0, 0, 1, 0).cast<Scalar>()));
  RotationImplementation halfZ =    RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(0, 0, 0, 1).cast<Scalar>()));


  RotationsTest() : tol(1e-6), X(Vector3::UnitX()), Y(Vector3::UnitY()), Z(Vector3::UnitZ()), Vgeneric(Vector3(2,10,-7)) {}
};

template <typename RotationImplementationPair>
struct RotationPairsTest : public ::testing::Test  {

  typedef typename RotationImplementationPair::first_t RotationImplementationA;
  typedef typename RotationImplementationPair::second_t RotationImplementationB;

  RotationsTest<RotationImplementationA> A;
  RotationsTest<RotationImplementationB> B;
};


typedef ::testing::Types<
    rot::AngleAxisPD,
    rot::AngleAxisPF
//    rot::RotationQuaternionPD,
//    rot::RotationQuaternionF,
//    rot::RotationMatrixD,
//    rot::RotationMatrixF,
//    rot::EulerAnglesXyzPD,
//    rot::EulerAnglesXyzF,
//    rot::EulerAnglesZyxPD,
//    rot::EulerAnglesZyxF,
//    rot::AngleAxisD,
//    rot::AngleAxisF,
//    rot::RotationQuaternionPD,
//    rot::RotationQuaternionF,
//    rot::RotationMatrixD,
//    rot::RotationMatrixF,
//    rot::EulerAnglesXyzPD,
//    rot::EulerAnglesXyzF,
//    rot::EulerAnglesZyxPD,
//    rot::EulerAnglesZyxF
> Types;

typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPD, rot::AngleAxisPD>
> TypePairs;

TYPED_TEST_CASE(RotationsTest, Types);

TYPED_TEST(RotationsTest, QuaternionToAxisAngle){

  ASSERT_EQ(this->Vgeneric, this->stdconstr.rotate(this->Vgeneric));

//  std::cout << this->tol << std::endl;

  for(auto & r : {TestFixture::halfX, TestFixture::halfY, TestFixture::halfZ}){
//    ASSERT_EQ(this->identity, r*r) << r*r; // TODO ASSERT_NEAR
//	  ASSERT_TRUE(rm::rotations::areNearlyEqual(this->identity, r*r, this->tol)); // TODO ASSERT_NEAR

//    ASSERT_EQ(this->X, (r*r).rotate(this->X));
//    ASSERT_EQ(this->Y, (r*r).rotate(this->Y));
//    ASSERT_EQ(this->Z, (r*r).rotate(this->Z));
  }

//  auto r1 = TestFixture::halfX;
//  std::cout << r1 << std::endl;
//  ASSERT_EQ(this->X, r1.rotate(this->X));
//  ASSERT_EQ(-this->Y, r1.rotate(this->Y)) << "CustomError: Expected: " << (-this->Y).transpose() << std::endl << "             Which is: "<< (r1.rotate(this->Y)).transpose();
//  ASSERT_EQ(-this->Z, r1.rotate(this->Z));


}


//TEST (RotationsTest, DISABLED_testRotationWrapper) {
//
//  sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));
//
//  for(int i=0; i<1e4; i++)
//  {
//	double p0 = sm::random::randLU(-100,100);
//	double p1 = sm::random::randLU(-100,100);
//	double p2 = sm::random::randLU(-100,100);
//	double p3 = sm::random::randLU(-100,100);
//	quat::QuaternionD q(p0,p1,p2,p3);
//	rot::RotationQuaternionPD rq(q.toUnitQuaternion());
//
//	rot::EulerAnglesXyzPD rpy(rq);
//	ASSERT_GE(rpy.roll(),  0);
//	ASSERT_LE(rpy.roll(),  M_PI);
//	ASSERT_GE(rpy.pitch(),-M_PI);
//	ASSERT_LE(rpy.pitch(), M_PI);
//	ASSERT_GE(rpy.yaw(),  -M_PI);
//	ASSERT_LE(rpy.yaw(),   M_PI);
//
//	rot::EulerAnglesZyxPD ypr(rq);
//	ASSERT_GE(ypr.yaw(),   0);
//	ASSERT_LE(ypr.yaw(),   M_PI);
//	ASSERT_GE(ypr.pitch(),-M_PI);
//	ASSERT_LE(ypr.pitch(), M_PI);
//	ASSERT_GE(ypr.roll(), -M_PI);
//	ASSERT_LE(ypr.roll(),  M_PI);
//
////	rot::EulerAnglesXyzPD rpy_unit;
////	rot::EulerAnglesXyzPD rpy_unit_pi;
////	rpy_unit_pi.roll()  = M_PI;
////	rpy_unit_pi.pitch() = M_PI;
////	rpy_unit_pi.yaw()   = M_PI;
////	rot::RotationMatrixD rot_unit(rpy_unit);
////	rot::RotationMatrixD rot_unit_pi(rpy_unit_pi);
////	ASSERT_MATRIX_NEAR(rot_unit.toImplementation(), rot_unit_pi.toImplementation(), 1e-6, "rot_unit");
//
////	rot::EulerAnglesXyzPD rpy_mod(rq);
////	rpy_mod.roll()  = rpy.roll()  + M_PI;
////	rpy_mod.pitch() = rpy.pitch() + M_PI;
////	rpy_mod.yaw()   = rpy.yaw()   + M_PI;
////	rot::RotationMatrixD rot(rpy);
////	rot::RotationMatrixD rot_mod(rpy_mod);
////	ASSERT_MATRIX_NEAR(rot.toImplementation(), rot_mod.toImplementation(), 1e-6, "rot");
//
////	rot::EulerAnglesXyzPD rpy_mod(rq);
////	if(rpy.pitch() >= M_PI/2)
////	{
////	  if(rpy.roll() >= 0) {
////		rpy.roll() -= M_PI;
////	  } else {
////		rpy.roll() += M_PI;
////	  }
////
////      rpy.pitch() = -(rpy.pitch()-M_PI);
////
////      if(rpy.yaw() >= 0) {
////    	rpy.yaw() -= M_PI;
////      } else {
////    	rpy.yaw() += M_PI;
////      }
////	}
////	else
////	if(rpy.pitch() < -M_PI/2)
////	{
////	  if(rpy.roll() >= 0) {
////		rpy.roll() -= M_PI;
////	  } else {
////		rpy.roll() += M_PI;
////	  }
////
////	  rpy.pitch() = -(rpy.pitch()+M_PI);
////
////	  if(rpy.yaw() >= 0) {
////		rpy.yaw() -= M_PI;
////	  } else {
////		rpy.yaw() += M_PI;
////	  }
////	}
////	rot::RotationMatrixD rot(rpy);
////	rot::RotationMatrixD rot_mod(rpy_mod);
////	ASSERT_MATRIX_NEAR(rot.toImplementation(), rot_mod.toImplementation(), 1e-6, "rot");
//
//
////	rot::EulerAnglesZyxPD ypr_mod(rq);
////	if(ypr.pitch() >= M_PI/2)
////	{
////	  if(ypr.yaw() >= 0) {
////		ypr.yaw() -= M_PI;
////	  } else {
////		ypr.yaw() += M_PI;
////	  }
////
////      ypr.pitch() = -(ypr.pitch()-M_PI);
////
////	  if(ypr.roll() >= 0) {
////		ypr.roll() -= M_PI;
////	  } else {
////		ypr.roll() += M_PI;
////	  }
////	}
////	else
////	if(ypr.pitch() < -M_PI/2)
////	{
////	  if(ypr.yaw() >= 0) {
////		ypr.yaw() -= M_PI;
////	  } else {
////		ypr.yaw() += M_PI;
////	  }
////
////	  ypr.pitch() = -(ypr.pitch()+M_PI);
////
////	  if(ypr.roll() >= 0) {
////		ypr.roll() -= M_PI;
////	  } else {
////		ypr.roll() += M_PI;
////	  }
////	}
////	rot::RotationMatrixD rot(ypr);
////	rot::RotationMatrixD rot_mod(ypr_mod);
////	ASSERT_MATRIX_NEAR(rot.toImplementation(), rot_mod.toImplementation(), 1e-6, "rot");
//  }
//}




TEST (RotationsTest, testRotationVarious ) {

  // todo:
  // go through todos in source code
  // range wrapper for angle axis and euler angles etc -> only in get functions and ==
  // test
  // interaction double - float
  // exponential function
  // doxygen documentation
  // copy paste for transformations

  rot::AngleAxisPD a1 = rot::AngleAxisPD(Eigen::AngleAxisd(1,Eigen::Vector3d(1,0,0)));
//  rot::RotationQuaternionPD q1 = a1; // calls q1(a1) implicitely
//  rot::RotationMatrixPD R1 = a1; // calls R1(a1) implicitely

  rot::RotationQuaternionPD q2(a1);
  rot::RotationMatrixPD R2(a1);
  rot::AngleAxisPD a2(a1);
  rot::EulerAnglesXyzPD xyz2(a1);
  rot::EulerAnglesZyxPD zyx2(a1);
//
  rot::RotationQuaternionPD q3;
  rot::RotationMatrixPD R3;
  rot::EulerAnglesXyzPD xyz3;
  rot::EulerAnglesZyxPD zyx3;
  std::cout << "\n";
  a1 = R3;
  a1 = q3;
  a1 = a2;
  a1 = xyz2;
  a1 = zyx2;
  q3 = a1;
  q3 = q2;
  q3 = R2;
  q3 = xyz2;
  q3 = zyx2;
  R3 = a1;
  R3 = R2;
  R3 = q3;
  R3 = xyz2;
  R3 = zyx2;
  xyz3 = a1;
  xyz3 = q2;
  xyz3 = R2;
  xyz3 = xyz2;
  xyz3 = zyx2;
  zyx3 = a1;
  zyx3 = q2;
  zyx3 = R2;
  zyx3 = xyz2;
  zyx3 = zyx2;

//  rot::EulerAnglesXyzPD rpy1(a1);
//  rot::EulerAnglesZyxPD ypr1(a1);
//
//
//  typedef Eigen::Matrix<double,3,1> Vector3d;
//
//  rot::AngleAxisPD a0(1,1,0,0);
//  rot::RotationQuaternionPD q0(1,0,0,0);
//  rot::RotationMatrixPD R0(0,-1,0,1,0,0,0,0,1);
//  rot::EulerAnglesXyzPD rpy0(1,-2,3);
//  rot::EulerAnglesZyxPD ypr0(-1,3,2);
//
//  Eigen::Matrix<double, 3, 1> v;
//  v << 1,2,3;
//  Eigen::Matrix<double, 3, 5> M;
//  M << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15;
//
//  std::cout << a0 << std::endl;
//  std::cout << q0 << std::endl;
//  std::cout << R0 << std::endl;
//  std::cout << rpy0 << std::endl;
//  std::cout << ypr0 << std::endl;
//  std::cout << std::endl;
//
//  std::cout << a0*a0 << std::endl;
//  std::cout << q0*q0 << std::endl;
//  std::cout << R0*R0 << std::endl;
//  std::cout << rpy0*rpy0 << std::endl;
//  std::cout << ypr0*ypr0 << std::endl;
//  std::cout << std::endl;
//
//  std::cout << a0*q0 << std::endl;
//  std::cout << q0*q0 << std::endl;
//  std::cout << R0*q0 << std::endl;
//  std::cout << rpy0*q0 << std::endl;
//  std::cout << ypr0*q0 << std::endl;
//  std::cout << std::endl;
//
//  std::cout << a0.inverted() << std::endl;
//  std::cout << q0.inverted() << std::endl;
//  std::cout << R0.inverted() << std::endl;
//  std::cout << rpy0.inverted() << std::endl;
//  std::cout << ypr0.inverted() << std::endl;
//  std::cout << std::endl;
//
//  std::cout << (a0==a0) << std::endl;
//  std::cout << (q0==q0) << std::endl;
//  std::cout << (R0==R0) << std::endl;
//  std::cout << (rpy0==rpy0) << std::endl;
//  std::cout << (ypr0==ypr0) << std::endl;
//  std::cout << std::endl;
//
//  std::cout << (a0==R0) << std::endl;
//  std::cout << (q0==rpy0) << std::endl;
//  std::cout << (R0==q0) << std::endl;
//  std::cout << (rpy0==ypr0) << std::endl;
//  std::cout << (ypr0==a0) << std::endl;
//  std::cout << std::endl;
//
//  std::cout << a0.rotate(v) << std::endl;
//  std::cout << a0.rotate(M) << std::endl;
//  std::cout << q0.rotate(v) << std::endl;
//  std::cout << q0.rotate(M) << std::endl;
//  std::cout << R0.rotate(v) << std::endl;
//  std::cout << R0.rotate(M) << std::endl;
//  std::cout << rpy0.rotate(v) << std::endl;
//  std::cout << rpy0.rotate(M) << std::endl;
//  std::cout << ypr0.rotate(v) << std::endl;
//  std::cout << ypr0.rotate(M) << std::endl;
//  std::cout << std::endl;
//
//  std::cout << a0.inverseRotate(v) << std::endl;
//  std::cout << a0.inverseRotate(M) << std::endl;
//  std::cout << q0.inverseRotate(v) << std::endl;
//  std::cout << q0.inverseRotate(M) << std::endl;
//  std::cout << R0.inverseRotate(v) << std::endl;
//  std::cout << R0.inverseRotate(M) << std::endl;
//  std::cout << rpy0.inverseRotate(v) << std::endl;
//  std::cout << rpy0.inverseRotate(M) << std::endl;
//  std::cout << ypr0.inverseRotate(v) << std::endl;
//  std::cout << ypr0.inverseRotate(M) << std::endl;
//  std::cout << std::endl;
//
//  std::cout << a0.setIdentity() << std::endl;
//  std::cout << (a0 = q0.setIdentity()) << std::endl;
//  std::cout << R0.setIdentity() << std::endl;
//  std::cout << rpy0.setIdentity() << std::endl;
//  std::cout << ypr0.setIdentity() << std::endl;
//  std::cout << std::endl;
//
//  std::cout << q0.w() << std::endl;
//  std::cout << q0.x() << std::endl;
//  std::cout << q0.y() << std::endl;
//  std::cout << q0.z() << std::endl;
//  std::cout << std::endl;
//
//  std::cout << a0.angle() << std::endl;
//  std::cout << a0.axis() << std::endl;
//  a0.angle() = 2;
//  a0.axis() = Vector3d(0,0,1);
//  std::cout << a0 << std::endl;
//  std::cout << std::endl;
//
//  std::cout << R0.matrix() << std::endl;
//  std::cout << R0.determinant() << std::endl;
//  std::cout << std::endl;
//
//  std::cout << rpy0.roll() << std::endl;
//  std::cout << rpy0.pitch() << std::endl;
//  std::cout << rpy0.yaw() << std::endl;
//  std::cout << std::endl;
//
//  std::cout << ypr0.yaw() << std::endl;
//  std::cout << ypr0.pitch() << std::endl;
//  std::cout << ypr0.roll() << std::endl;
//  std::cout << std::endl;


}

TEST (RotationsTest, testActivePassive) {

  rot::RotationQuaternionPD qp1(0,1,0,0);
  rot::RotationQuaternionAD qa1 = qp1.getActive();
  rot::RotationQuaternionPD qp2 = qa1.getPassive();
//  rot::RotationQuaternionPD qp3 = qp2.getPassive();

}




