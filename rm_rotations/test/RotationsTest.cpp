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


TEST (RotationsTest, testRotationMatrixFromKardanAngles ) {
  using namespace Eigen;
  using namespace rm::rotations;

  // random seed
    sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));

    Quaterniond p_BI = Quaterniond(cos(1),0,0,sin(1));
//     std::cout << getRPYFromQuaternion(p_BI).transpose() << std::endl;
//     std::cout << getYPRFromQuaternion(p_BI).transpose() << std::endl;
//
//     Vector3d rpy_BI2 = Vector3d(0,0,2);
//     Quaterniond test = getQuaternionFromRPY(rpy_BI2);
//     std::cout << test.w() << test.x() << test.y() << test.z() << std::endl;
//
//     Matrix3d R_BI = Matrix3d::Zero();
//     R_BI << cos(M_PI/6), sin(M_PI/6), 0, -sin(M_PI/6), cos(M_PI/6), 0, 0, 0, 1;
//     std::cout << getRPYFromRotationMatrix(R_BI) << std::endl;
//     std::cout << getAngleAxisFromRotationMatrix(R_BI).angle() << std::endl;
//     std::cout << getAngleAxisFromRotationMatrix(R_BI).axis().transpose() << std::endl;
//
//     Vector3d rpy_BI2 = Vector3d(0,0,1);
//     std::cout << getRotationMatrixFromRPY(rpy_BI2) << std::endl;
//
//     std::cout << getAngleAxisFromRPY(rpy_BI2).axis() << std::endl << getAngleAxisFromRPY(rpy_BI2).angle() << std::endl;

//    std::cout << getRotationMatrixFromAngleAxis(AngleAxisd(M_PI/6,Vector3d(0,0,1))) << std::endl;

//     std::cout << getRotationMatrixFromQuaternion(Quaterniond(cos(M_PI/6),0,0,sin(M_PI/6))) << std::endl;

//    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).w() << std::endl;
//    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).x() << std::endl;
//    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).y() << std::endl;
//    std::cout << getQuaternionFromAngleAxis(AngleAxisd(M_PI/3,Vector3d(0,0,1))).z() << std::endl;




    for(int i=0; i<10000; i++)
    {
      // create random rotation
      const double roll = sm::random::randLU(-M_PI,M_PI);
      const double pitch = sm::random::randLU(-M_PI/2,M_PI/2);
      const double yaw = sm::random::randLU(-M_PI,M_PI);
      const Vector3d rpy_BI0 = Vector3d(roll,pitch,yaw);

      ASSERT_DOUBLE_MX_EQ(rpy_BI0, getRPYFromAngleAxis(getAngleAxisFromRPY(rpy_BI0)), 1e-6, "rpy1");
      ASSERT_DOUBLE_MX_EQ(rpy_BI0, getRPYFromQuaternion(getQuaternionFromRPY(rpy_BI0)), 1e-6, "rpy2");
      ASSERT_DOUBLE_MX_EQ(rpy_BI0, getRPYFromRotationMatrix(getRotationMatrixFromRPY(rpy_BI0)), 1e-6, "rpy3");

      const AngleAxisd aa_BI0 = getAngleAxisFromRPY(rpy_BI0);
      const Quaterniond p_BI0 = getQuaternionFromRPY(rpy_BI0);
      const Matrix3d R_BI0 = getRotationMatrixFromRPY(rpy_BI0);
      const Vector3d ypr_BI0 = getYPRFromRPY(rpy_BI0);

      ASSERT_DOUBLE_MX_EQ(aa_BI0.axis()*aa_BI0.angle(), getAngleAxisFromQuaternion(getQuaternionFromAngleAxis(aa_BI0)).axis()*getAngleAxisFromQuaternion(getQuaternionFromAngleAxis(aa_BI0)).angle(), 1e-6, "aa1");

      ASSERT_DOUBLE_MX_EQ(R_BI0, getRotationMatrixFromAngleAxis(getAngleAxisFromRotationMatrix(R_BI0)), 1e-6, "rot1");
      ASSERT_DOUBLE_MX_EQ(R_BI0, getRotationMatrixFromQuaternion(getQuaternionFromRotationMatrix(R_BI0)), 1e-6, "rot2");

      ASSERT_DOUBLE_MX_EQ(ypr_BI0, getYPRFromAngleAxis(getAngleAxisFromYPR(ypr_BI0)), 1e-6, "ypr1");
      ASSERT_DOUBLE_MX_EQ(ypr_BI0, getYPRFromQuaternion(getQuaternionFromYPR(ypr_BI0)), 1e-6, "ypr2");
      ASSERT_DOUBLE_MX_EQ(ypr_BI0, getYPRFromRotationMatrix(getRotationMatrixFromYPR(ypr_BI0)), 1e-6, "ypr3");
      ASSERT_DOUBLE_MX_EQ(ypr_BI0, getYPRFromRPY(getRPYFromYPR(ypr_BI0)), 1e-6, "ypr4");

//      std::cout << aa_BI0.angle() << ", " << getInverseAngleAxis(aa_BI0).angle() << std::endl;
//      std::cout << aa_BI0.axis().transpose() << ", " << getInverseAngleAxis(aa_BI0).axis().transpose() << std::endl;
//      std::cout << std::endl;
//
//      std::cout << p_BI0.w() << ", " << getInverseQuaternion(p_BI0).w() << std::endl;
//      std::cout << p_BI0.x() << ", " << getInverseQuaternion(p_BI0).x() << std::endl;
//      std::cout << p_BI0.y() << ", " << getInverseQuaternion(p_BI0).y() << std::endl;
//      std::cout << p_BI0.z() << ", " << getInverseQuaternion(p_BI0).z() << std::endl;
//      std::cout << std::endl;
//
//      std::cout << R_BI0 << std::endl;
//      std::cout << getInverseRotationMatrix(R_BI0) << std::endl;
//      std::cout << std::endl;
//
//      std::cout << rpy_BI0.transpose() << std::endl;
//      std::cout << getInverseRPY(rpy_BI0).transpose() << std::endl;
//      std::cout << getInverseRPY(getInverseRPY(rpy_BI0)).transpose() << std::endl;
//      std::cout << std::endl;
//
//      std::cout << rpy_BI0.transpose() << std::endl;
//      std::cout << getInverseRPY2(rpy_BI0).transpose() << std::endl;
//      std::cout << getInverseRPY2(getInverseRPY2(rpy_BI0)).transpose() << std::endl;
//      std::cout << std::endl;
//
//      std::cout << ypr_BI0.transpose() << std::endl;
//      std::cout << getInverseYPR(ypr_BI0).transpose() << std::endl;
//      std::cout << getInverseYPR(getInverseYPR(ypr_BI0)).transpose() << std::endl;
//      std::cout << std::endl;


//      std::cout << p_BI0.w() << ", " << p_BI0.x() << ", " << p_BI0.y() << ", " << p_BI0.z() << std::endl;
//      std::cout << getAngleAxisFromQuaternion(p_BI0).angle() << ", " << getAngleAxisFromQuaternion(p_BI0).axis().transpose() << std::endl;

//      const double chi = 2*acos(p_BI0.w());
//      const Vector3d n = Vector3d(p_BI0.x()/sin(chi/2),p_BI0.y()/sin(chi/2),p_BI0.z()/sin(chi/2));
//      std::cout << getAngleAxisFromQuaternion(p_BI0).angle() << ", " << chi << std::endl;
//      std::cout << getAngleAxisFromQuaternion(p_BI0).axis().transpose() << ", " << n.transpose() << std::endl;


//      ASSERT_DOUBLE_MX_EQ(A_BI, A_BI2, 1e-6, "RotationMatrix");
    }
}
