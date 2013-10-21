#include <gtest/gtest.h>

#include "rm/rotations/Rotations.hpp"

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

TEST (RotationsTest, testRotationFunctions ) {
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

      ASSERT_DOUBLE_MX_EQ(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromQuaternion(p_IB0).axis()*getAngleAxisFromQuaternion(p_IB0).angle(), 1e-4, "aa1"); // ok
      ASSERT_DOUBLE_MX_EQ(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromTransformationMatrix(A_BI0).axis()*correctRangeAngle(getAngleAxisFromTransformationMatrix(A_BI0).angle()), 1e-5, "aa2"); // ok
      ASSERT_DOUBLE_MX_EQ(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromRPY(rpy_IB0).axis()*correctRangeAngle(getAngleAxisFromRPY(rpy_IB0).angle()), 1e-4, "aa3"); // ok
      ASSERT_DOUBLE_MX_EQ(aa_IB0.axis()*aa_IB0.angle(), getAngleAxisFromYPR(ypr_IB0).axis()*correctRangeAngle(getAngleAxisFromYPR(ypr_IB0).angle()), 1e-4, "aa4"); // ok

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
