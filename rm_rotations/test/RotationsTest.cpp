#include <gtest/gtest.h>

#include "rm/common/Common.hpp"
#include "rm/rotations/RotationEigenFunctions.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sm/eigen/gtest.hpp>
#include <sm/random.hpp>
#include <sm/timing/Timer.hpp>

#include <limits>
#include <iostream>
#include <random>

  // use a timer
  typedef sm::timing::Timer MyTimerType;
  // use no timer
  //typedef sm::timing::DummyTimer MyTimerType;

TEST (RotationsTest, DISABLED_testEigenEulerAngleRange ) {
  using namespace Eigen;
  using namespace rm::rotations;

  std::cout << getRPYFromQuaternion(getQuaternionFromRPY(Vector3d(M_PI,0,0))).transpose() << std::endl;
  std::cout << getRPYFromQuaternion(getQuaternionFromRPY(Vector3d(-M_PI,0,0))).transpose() << std::endl;
  std::cout << getRPYFromQuaternion(getQuaternionFromRPY(Vector3d(0,M_PI/2,0))).transpose() << std::endl;
  std::cout << getRPYFromQuaternion(getQuaternionFromRPY(Vector3d(0,-M_PI/2,0))).transpose() << std::endl;
  std::cout << getRPYFromQuaternion(getQuaternionFromRPY(Vector3d(0,0,M_PI))).transpose() << std::endl;
  std::cout << getRPYFromQuaternion(getQuaternionFromRPY(Vector3d(0,0,-M_PI))).transpose() << std::endl;

  std::cout << std::endl;

  double eps = 0.001;

  std::cout << getRPYFromAngleAxis(AngleAxisd(M_PI,Vector3d(1,0,0))).transpose() << std::endl;
  std::cout << getRPYFromAngleAxis(AngleAxisd(-M_PI,Vector3d(1,0,0))).transpose() << std::endl;
  std::cout << getRPYFromAngleAxis(AngleAxisd(M_PI,Vector3d(0,1,0))).transpose() << std::endl;
  std::cout << getRPYFromAngleAxis(AngleAxisd(-M_PI,Vector3d(0,1,0))).transpose() << std::endl;
  std::cout << getRPYFromAngleAxis(AngleAxisd(M_PI,Vector3d(0,0,1))).transpose() << std::endl;
  std::cout << getRPYFromAngleAxis(AngleAxisd(-M_PI,Vector3d(0,0,1))).transpose() << std::endl;
}

TEST (RotationsTest, DISABLED_testWrapAngle ) {
  using namespace rm::common;
  sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));

  for(int i=0; i<1e6; i++)
  {
    double angle = sm::random::randLU(-100,100);

    EXPECT_EQ(wrapPosNegPI(angle), wrapAngle(angle,-M_PI,M_PI));
  }
}

TEST (RotationsTest, DISABLED_testQuaternionMultiplication ) {
  using namespace Eigen;
  using namespace rm::rotations;
  sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));

  Vector3d n1 = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
  n1.normalize();
  AngleAxisd aa_AI = AngleAxisd(sm::random::randLU(-M_PI,M_PI),n1);
  Quaterniond p_AI = Quaterniond(aa_AI);

  Vector3d n2 = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
  n2.normalize();
  AngleAxisd aa_BA = AngleAxisd(sm::random::randLU(-M_PI,M_PI),n2);
  Quaterniond p_BA = Quaterniond(aa_BA);

  Quaterniond p_BI1 = p_BA*p_AI;

  double w = p_BA.w()*p_AI.w() - p_BA.x()*p_AI.x() - p_BA.y()*p_AI.y() - p_BA.z()*p_AI.z();
  double x = p_BA.w()*p_AI.x() + p_AI.w()*p_BA.x() + p_BA.y()*p_AI.z() - p_BA.z()*p_AI.y();
  double y = p_BA.w()*p_AI.y() + p_AI.w()*p_BA.y() + p_BA.z()*p_AI.x() - p_BA.x()*p_AI.z();
  double z = p_BA.w()*p_AI.z() + p_AI.w()*p_BA.z() + p_BA.x()*p_AI.y() - p_BA.y()*p_AI.x();
  Quaterniond p_BI2 = Quaterniond(w,x,y,z);

  ASSERT_DOUBLE_MX_EQ(quaternionToVector(p_BI1), quaternionToVector(p_BI2), 1e-6, "p");

  Vector3d I_r = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));

  Vector3d B_r1 = (p_BA * p_AI) * I_r;
  Vector3d B_r2 = p_BA * (p_AI * I_r);

  ASSERT_DOUBLE_MX_EQ(B_r1, B_r2, 1e-6, "p");
}

TEST (RotationsTest, DISABLED_testRotationFunctions ) {
  using namespace Eigen;
  using namespace rm::rotations;

  // random seed
    sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));

//    Quaterniond p_IB = Quaterniond(cos(M_PI/12),0,0,sin(M_PI/12));
//     std::cout << getRPYFromQuaternion(p_IB).transpose() << std::endl;
//     std::cout << getYPRFromQuaternion(p_IB).transpose() << std::endl;
//
//     Vector3d rpy_IB2 = Vector3d(0,0,2);
//     Quaterniond test = getQuaternionFromRPY(rpy_IB2);
//     std::cout << test.w() << test.x() << test.y() << test.z() << std::endl;
//
//     Matrix3d A_BI = Matrix3d::Zero();
//     A_BI << cos(M_PI/6), sin(M_PI/6), 0, -sin(M_PI/6), cos(M_PI/6), 0, 0, 0, 1;
//     std::cout << getRPYFromTransformationMatrix(A_BI) << std::endl;
//     std::cout << getAngleAxisFromTransformationMatrix(A_BI).angle() << std::endl;
//     std::cout << getAngleAxisFromTransformationMatrix(A_BI).axis().transpose() << std::endl;
//
//     Vector3d rpy_IB2 = Vector3d(0,0,1);
//     std::cout << getTransformationMatrixFromRPY(rpy_IB2) << std::endl;
//
//     std::cout << getAngleAxisFromRPY(rpy_IB2).axis() << std::endl << getAngleAxisFromRPY(rpy_IB2).angle() << std::endl;

//    std::cout << getTransformationMatrixFromAngleAxis(AngleAxisd(-M_PI/6,Vector3d(0,0,1))) << std::endl;
//    std::cout << getTransformationMatrixFromQuaternion(Quaterniond(AngleAxisd(-M_PI/6,Vector3d(0,0,1)))) << std::endl;

//    Vector3d I_r = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
//    Vector3d I_r_p =
//    std::cout << getTransformationMatrixFromQuaternion(Quaterniond(AngleAxisd(M_PI/6,Vector3d(0,0,1))))*I_r << std::endl;

//     std::cout << getTransformationMatrixFromQuaternion(Quaterniond(cos(M_PI/6),0,0,sin(M_PI/6))) << std::endl;

//    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).w() << std::endl;
//    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).x() << std::endl;
//    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).y() << std::endl;
//    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).z() << std::endl;

//    std::cout << getRPYFromQuaternion(Quaterniond(cos(M_PI/2),0,sin(M_PI/2),0)) << std::endl;


    for(int i=0; i<1e5; i++)
    {
      Vector3d n0 = Vector3d(sm::random::randLU(-100,100),sm::random::randLU(-100,100),sm::random::randLU(-100,100));
      n0.normalize();
      AngleAxisd aa_IB0 = AngleAxisd(sm::random::randLU(-M_PI,M_PI),n0);

//      Vector3d rpy_IB = Vector3d(sm::random::randLU(-M_PI,M_PI),sm::random::randLU(-M_PI,M_PI),sm::random::randLU(-M_PI,M_PI));
//      Vector3d rpy_IB = Vector3d(sm::random::randLU(-M_PI,M_PI),M_PI/2,sm::random::randLU(-M_PI,M_PI));
//      AngleAxisd aa_IB0 = getAngleAxisFromRPY(rpy_IB);

      Quaterniond p_IB0 = Quaterniond(aa_IB0);
      Matrix3d A_BI0 = p_IB0.toRotationMatrix();
      Vector3d rpy_IB0 = getRPYFromTransformationMatrix(A_BI0);
      Vector3d ypr_IB0 = getYPRFromTransformationMatrix(A_BI0);

//      std::cout << std::endl << std::endl;
//      std::cout << "angle = " << aa_IB0.angle() << ", axis = " << aa_IB0.axis().transpose() << std::endl;
//      std::cout << "quaternion = " << quaternionToVector(p_IB0).transpose() << std::endl;
//      std::cout << "trafo = " << std::endl << A_BI0 << std::endl;
//      std::cout << "rpy = " << rpy_IB0.transpose() << std::endl;
//      std::cout << "ypr = " << ypr_IB0.transpose() << std::endl;

//      std::cout << std::endl << std::endl;
//      std::cout << "p0 = " << quaternionToVector(p_IB0).transpose() << std::endl;
//      std::cout << "p1 = " << quaternionToVector(getQuaternionFromTransformationMatrix(getTransformationMatrixFromRPY(rpy_IB0))).transpose() << std::endl;
//      std::cout << "p2 = " << quaternionToVector(getQuaternionFromRPY(rpy_IB0)).transpose() << std::endl;

      ASSERT_DOUBLE_MX_EQ(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromQuaternion(p_IB0).axis()*getAngleAxisFromQuaternion(p_IB0).angle(), 3e-4, "aa1"); // ok
      ASSERT_DOUBLE_MX_EQ(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromTransformationMatrix(A_BI0).axis()*getAngleAxisFromTransformationMatrix(A_BI0).angle(), 3e-4, "aa2"); // ok
      ASSERT_DOUBLE_MX_EQ(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromRPY(rpy_IB0).axis()*getAngleAxisFromRPY(rpy_IB0).angle(), 3e-4, "aa3"); // ok
      ASSERT_DOUBLE_MX_EQ(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromYPR(ypr_IB0).axis()*getAngleAxisFromYPR(ypr_IB0).angle(), 3e-4, "aa4"); // ok

      ASSERT_DOUBLE_MX_EQ(quaternionToVector(p_IB0), quaternionToVector(getQuaternionFromAngleAxis(aa_IB0)), 1e-6, "quat1"); // ok
      ASSERT_DOUBLE_MX_EQ(quaternionToVector(p_IB0), quaternionToVector(getQuaternionFromTransformationMatrix(A_BI0)), 1e-6, "quat2"); // ok
      ASSERT_DOUBLE_MX_EQ(quaternionToVector(p_IB0), quaternionToVector(getQuaternionFromRPY(rpy_IB0)), 1e-6, "quat3"); // ok
      ASSERT_DOUBLE_MX_EQ(quaternionToVector(p_IB0), quaternionToVector(getQuaternionFromYPR(ypr_IB0)), 1e-6, "quat4"); // ok

      ASSERT_DOUBLE_MX_EQ(A_BI0, getTransformationMatrixFromAngleAxis(aa_IB0), 1e-6, "trafo1"); // ok
      ASSERT_DOUBLE_MX_EQ(A_BI0, getTransformationMatrixFromQuaternion(p_IB0), 1e-6, "trafo2"); // ok
      ASSERT_DOUBLE_MX_EQ(A_BI0, getTransformationMatrixFromRPY(rpy_IB0), 1e-6, "trafo3"); // ok
      ASSERT_DOUBLE_MX_EQ(A_BI0, getTransformationMatrixFromYPR(ypr_IB0), 1e-6, "trafo4"); // ok

      ASSERT_DOUBLE_MX_EQ(rpy_IB0, getRPYFromAngleAxis(aa_IB0), 1e-6, "rpy1"); // ok
      ASSERT_DOUBLE_MX_EQ(rpy_IB0, getRPYFromQuaternion(p_IB0), 1e-6, "rpy2"); // ok
      ASSERT_DOUBLE_MX_EQ(rpy_IB0, getRPYFromTransformationMatrix(A_BI0), 1e-6, "rpy3"); // ok
      ASSERT_DOUBLE_MX_EQ(rpy_IB0, getRPYFromYPR(ypr_IB0), 1e-6, "rpy4"); // ok

      ASSERT_DOUBLE_MX_EQ(ypr_IB0, getYPRFromAngleAxis(aa_IB0), 1e-6, "ypr1"); // ok
      ASSERT_DOUBLE_MX_EQ(ypr_IB0, getYPRFromQuaternion(p_IB0), 1e-6, "ypr2"); // ok
      ASSERT_DOUBLE_MX_EQ(ypr_IB0, getYPRFromTransformationMatrix(A_BI0), 1e-6, "ypr3"); // ok
      ASSERT_DOUBLE_MX_EQ(ypr_IB0, getYPRFromRPY(rpy_IB0), 1e-6, "ypr4"); // ok
    }
}


#include <rm/quaternions/QuaternionEigen.hpp>
#include <rm/rotations/RotationEigen.hpp>

namespace rot = rm::rotations::eigen_implementation;

template <typename RotationImplementation>
struct RotationsTest : public ::testing::Test  {
  typedef typename RotationImplementation::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
//  typedef Eigen::Matrix<double,1,1> Vector1d;

//  typedef typename GenericScalar_::Scalar PrimScalar;
//  typedef GenericScalarExpression<PrimScalar> TestGenericScalarExpression;
//  typedef GenericScalar_ TestGenericScalar;

//  static PrimScalar getRandScalar() {
//    return PrimScalar(sm::random::rand() * 10.0);
//  }
  RotationImplementation identity = RotationImplementation(rot::RotationQuaternion<Scalar>(Eigen::Quaterniond(1, 0, 0, 0).cast<Scalar>()));
  RotationImplementation halfX =    RotationImplementation(rot::RotationQuaternion<Scalar>(Eigen::Quaterniond(0, 1, 0, 0).cast<Scalar>()));
  RotationImplementation halfY =    RotationImplementation(rot::RotationQuaternion<Scalar>(Eigen::Quaterniond(0, 0, 1, 0).cast<Scalar>()));
  RotationImplementation halfZ =    RotationImplementation(rot::RotationQuaternion<Scalar>(Eigen::Quaterniond(0, 0, 0, 1).cast<Scalar>()));


  RotationsTest() : X(Vector3::UnitX()), Y(Vector3::UnitY()), Z(Vector3::UnitZ()) {}
  Vector3 X, Y, Z;
};

template <typename RotationImplementationPair>
struct RotationPairsTest : public ::testing::Test  {

  typedef typename RotationImplementationPair::first_t RotationImplementationA;
  typedef typename RotationImplementationPair::second_t RotationImplementationB;

  RotationsTest<RotationImplementationA> A;
  RotationsTest<RotationImplementationB> B;
};


typedef ::testing::Types<
    rot::AngleAxisD,
    rot::AngleAxisF,
    rot::RotationQuaternionD,
    rot::RotationQuaternionF,
    rot::RotationMatrixD,
    rot::RotationMatrixF,
    rot::EulerAnglesRPYD,
    rot::EulerAnglesRPYF,
    rot::EulerAnglesYPRD,
    rot::EulerAnglesYPRF
> Types;

typedef ::testing::Types<
    std::pair<rot::RotationQuaternionD, rot::AngleAxisD>
> TypePairs;

TYPED_TEST_CASE(RotationsTest, Types);

TYPED_TEST(RotationsTest, DISABLED_QuaternionToAxisAngle){
  for(auto & r : {TestFixture::halfX, TestFixture::halfY, TestFixture::halfZ}){
    ASSERT_EQ(this->identity, r*r) << r*r; // TODO ASSERT_NEAR

    ASSERT_EQ(this->X, (r*r).rotate(this->X));
    ASSERT_EQ(this->Y, (r*r).rotate(this->Y));
    ASSERT_EQ(this->Z, (r*r).rotate(this->Z));
  }

  auto r1 = TestFixture::halfX;
  std::cout << r1 << std::endl;
  ASSERT_EQ(this->X, r1.rotate(this->X));
  ASSERT_EQ(-this->Y, r1.rotate(this->Y)) << "CustomError: Expected: " << (-this->Y).transpose() << std::endl << "             Which is: "<< (r1.rotate(this->Y)).transpose();
  ASSERT_EQ(-this->Z, r1.rotate(this->Z));




//  rot::RotationQuaternionD uqd(Eigen::Quaterniond(0,1,0,0));
////  rot::UnitQuaternionF uqf(Eigen::Quaterniond(0,1,0,0));
//  rot::RotationMatrixD Rd(Eigen::Quaterniond(0,1,0,0).toRotationMatrix());
//
//  rot::AngleAxisD ad(uqd);
//  rot::AngleAxisF af(uqd * uqd);
//
//  rot::RotationQuaternionD Id(1, 0, 0, 0);
//  rot::RotationQuaternionD I(0, 1, 0, 0);
//  rot::RotationQuaternionD J(0, 0, 1, 0);
//  rot::RotationQuaternionD K(0, 0, 0, 1);
//  rot::RotationQuaternionD qd(ad);
//  rot::RotationQuaternionD q5 = I;
//  rot::RotationQuaternionD q6(I);
//
//  rot::AngleAxisD a3 = ad*ad;
//  rot::RotationQuaternionD q7 = qd*qd;
//  rot::AngleAxisD a2 = ad*qd;
//
//  Eigen::Vector3d v(0, 1, 0);
//
//  Eigen::Vector3d vrot = (uqd*uqd).rotate(v);
//  Eigen::Vector3d vrot2 = (Rd*Rd).rotate(v);
//  Eigen::Vector3d vrot3 = ad.rotate(v);
//  Eigen::Vector3d vrot4 = (ad*ad).rotate(v);
//
//  std::cout << vrot.transpose() << std::endl;
//  std::cout << vrot2.transpose() << std::endl;
}

TEST (RotationsTest, testRotationVarious ) {

  // todo:
  // go through todos in source code
  // range wrapper for angle axis and euler angles etc -> only in get functions and ==
  // debug: check unitquaternion length
  // test
  // interaction double - float
  // exponential function
  // doxygen documentation
  // copy paste for transformations
  // conjugate(d), inverse(d)

  rot::AngleAxis<double> a1 = rot::AngleAxis<double>(Eigen::AngleAxisd(1,Eigen::Vector3d(1,0,0)));
//  rot::RotationQuaternion<double> q1 = a1; // calls q1(a1) implicitely
//  rot::RotationMatrix<double> R1 = a1; // calls R1(a1) implicitely

  rot::RotationQuaternion<double> q2(a1);
  rot::RotationMatrix<double> R2(a1);
  rot::RotationMatrix<double> a2(a1);

  rot::RotationQuaternion<double> q3;
  rot::RotationMatrix<double> R3;
  std::cout << "\n";
  q3 = a1;
  q3 = q2;
  q3 = R2;
  R3 = a1;
  R3 = R2;
  R3 = q3;
  a1 = R3;
  a1 = q3;
  a1 = a2;

  rot::EulerAnglesRPY<double> rpy1(a1);
  rot::EulerAnglesYPR<double> ypr1(a1);


  typedef Eigen::Matrix<double,3,1> Vector3d;

  rot::AngleAxis<double> a0(1,1,0,0);
  rot::RotationQuaternion<double> q0(1,0,0,0);
  rot::RotationMatrix<double> R0(0,1,0,1,0,0,0,0,1);
  rot::EulerAnglesRPY<double> rpy0(1,-2,3);
  rot::EulerAnglesYPR<double> ypr0(-1,3,2);

  Eigen::Matrix<double, 3, 1> v;
  v << 1,2,3;
  Eigen::Matrix<double, 3, 5> M;
  M << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15;

  std::cout << a0 << std::endl;
  std::cout << q0 << std::endl;
  std::cout << R0 << std::endl;
  std::cout << rpy0 << std::endl;
  std::cout << ypr0 << std::endl;
  std::cout << std::endl;

  std::cout << a0*a0 << std::endl;
  std::cout << q0*q0 << std::endl;
  std::cout << R0*R0 << std::endl;
  std::cout << rpy0*rpy0 << std::endl;
  std::cout << ypr0*ypr0 << std::endl;
  std::cout << std::endl;

  std::cout << a0*q0 << std::endl;
  std::cout << q0*q0 << std::endl;
  std::cout << R0*q0 << std::endl;
  std::cout << rpy0*q0 << std::endl;
  std::cout << ypr0*q0 << std::endl;
  std::cout << std::endl;

  std::cout << a0.inverse() << std::endl;
  std::cout << q0.inverse() << std::endl;
  std::cout << R0.inverse() << std::endl;
  std::cout << rpy0.inverse() << std::endl;
  std::cout << ypr0.inverse() << std::endl;
  std::cout << std::endl;

  std::cout << (a0==a0) << std::endl;
  std::cout << (q0==q0) << std::endl;
  std::cout << (R0==R0) << std::endl;
  std::cout << (rpy0==rpy0) << std::endl;
  std::cout << (ypr0==ypr0) << std::endl;
  std::cout << std::endl;

  std::cout << (a0==R0) << std::endl;
  std::cout << (q0==rpy0) << std::endl;
  std::cout << (R0==q0) << std::endl;
  std::cout << (rpy0==ypr0) << std::endl;
  std::cout << (ypr0==a0) << std::endl;
  std::cout << std::endl;

  std::cout << a0.rotate(v) << std::endl;
  std::cout << a0.rotate(M) << std::endl;
  std::cout << q0.rotate(v) << std::endl;
  std::cout << q0.rotate(M) << std::endl;
  std::cout << R0.rotate(v) << std::endl;
  std::cout << R0.rotate(M) << std::endl;
  std::cout << rpy0.rotate(v) << std::endl;
  std::cout << rpy0.rotate(M) << std::endl;
  std::cout << ypr0.rotate(v) << std::endl;
  std::cout << ypr0.rotate(M) << std::endl;
  std::cout << std::endl;

  std::cout << a0.inverserotate(v) << std::endl;
  std::cout << a0.inverserotate(M) << std::endl;
  std::cout << q0.inverserotate(v) << std::endl;
  std::cout << q0.inverserotate(M) << std::endl;
  std::cout << R0.inverserotate(v) << std::endl;
  std::cout << R0.inverserotate(M) << std::endl;
  std::cout << rpy0.inverserotate(v) << std::endl;
  std::cout << rpy0.inverserotate(M) << std::endl;
  std::cout << ypr0.inverserotate(v) << std::endl;
  std::cout << ypr0.inverserotate(M) << std::endl;
  std::cout << std::endl;

  std::cout << a0.setIdentity() << std::endl;
  std::cout << (a0 = q0.setIdentity()) << std::endl;
  std::cout << R0.setIdentity() << std::endl;
  std::cout << rpy0.setIdentity() << std::endl;
  std::cout << ypr0.setIdentity() << std::endl;
  std::cout << std::endl;

  std::cout << q0.w() << std::endl;
  std::cout << q0.x() << std::endl;
  std::cout << q0.y() << std::endl;
  std::cout << q0.z() << std::endl;
  std::cout << std::endl;

  std::cout << a0.angle() << std::endl;
  std::cout << a0.axis() << std::endl;
  a0.angle() = 2;
  a0.axis() = Vector3d(0,0,1);
  std::cout << a0 << std::endl;
  std::cout << std::endl;

  std::cout << R0.matrix() << std::endl;
  std::cout << std::endl;

  std::cout << rpy0.roll() << std::endl;
  std::cout << rpy0.pitch() << std::endl;
  std::cout << rpy0.yaw() << std::endl;
  std::cout << std::endl;

  std::cout << ypr0.yaw() << std::endl;
  std::cout << ypr0.pitch() << std::endl;
  std::cout << ypr0.roll() << std::endl;
  std::cout << std::endl;







}




