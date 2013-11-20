/*!
* @file 	  Transformations.hpp
* @author 	Michael Blösch, Peter Fankhauser, Christian Gehring, Remo Diethelm
* @date		  16 10, 2013
* @version 	1.0
* @ingroup 	rm
* @brief
*/
#ifndef RM_ROTATION_EIGEN_FUNCTIONS_HPP_
#define RM_ROTATION_EIGEN_FUNCTIONS_HPP_


#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rm/common/Common.hpp"

namespace rm {
namespace rotations {


// 1) Output: AngleAxis

template<typename T, typename TReturn = T>
static Eigen::AngleAxis<TReturn> getAngleAxisFromQuaternion(const Eigen::Quaternion<T>& p_BI)
{
  // Bad precision!
  return Eigen::AngleAxis<TReturn>(p_BI.template cast<TReturn>());
}

template<typename T, typename TReturn = T>
static Eigen::AngleAxis<TReturn> getAngleAxisFromTransformationMatrix(const Eigen::Matrix<T,3,3>& A_IB)
{
  // Bad precision!
  return Eigen::AngleAxis<TReturn>(A_IB.template cast<TReturn>());
}

template<typename T, typename TReturn = T>
static Eigen::AngleAxis<TReturn> getAngleAxisFromRotationMatrix(const Eigen::Matrix<T,3,3>& R_BI)
{
  // Bad precision!
  return Eigen::AngleAxis<TReturn>(R_BI.template cast<TReturn>());
}

template<typename T, typename TReturn = T>
static Eigen::AngleAxis<TReturn> getAngleAxisFromRPY(const Eigen::Matrix<T,3,1>& rpy_BI)
{
  // Bad precision!
  return Eigen::AngleAxis<TReturn>(
    Eigen::AngleAxis<TReturn>((TReturn)rpy_BI(0), Eigen::Matrix<TReturn, 3, 1>::UnitX()) *
    Eigen::AngleAxis<TReturn>((TReturn)rpy_BI(1), Eigen::Matrix<TReturn, 3, 1>::UnitY()) *
    Eigen::AngleAxis<TReturn>((TReturn)rpy_BI(2), Eigen::Matrix<TReturn, 3, 1>::UnitZ()));
}

template<typename T, typename TReturn = T>
static Eigen::AngleAxis<TReturn> getAngleAxisFromYPR(const Eigen::Matrix<T,3,1>& ypr_BI)
{
  // Bad precision!
  return Eigen::AngleAxis<TReturn>(
    Eigen::AngleAxis<TReturn>((TReturn)ypr_BI(0), Eigen::Matrix<TReturn, 3, 1>::UnitZ()) *
    Eigen::AngleAxis<TReturn>((TReturn)ypr_BI(1), Eigen::Matrix<TReturn, 3, 1>::UnitY()) *
    Eigen::AngleAxis<TReturn>((TReturn)ypr_BI(2), Eigen::Matrix<TReturn, 3, 1>::UnitX()));
}


// 2) Output: Quaternion

template<typename T, typename TReturn = T>
static Eigen::Quaternion<TReturn> getQuaternionFromAngleAxis(const Eigen::AngleAxis<T>& aa_BI)
{
  return Eigen::Quaternion<TReturn>(aa_BI.template cast<TReturn>());
}

template<typename T, typename TReturn = T>
static Eigen::Quaternion<TReturn> getQuaternionFromTransformationMatrix(const Eigen::Matrix<T,3,3>& A_IB)
{
//  // Untested
//  // Bad precision!
//  double w;
//  double x;
//  double y;
//  double z;
//  w = sqrt(1+mat(0,0)+mat(1,1)+mat(2,2))/2;
//  if(w>0.02){
//    x = 1/4/w*(mat(2,1)-mat(1,2));
//    y = 1/4/w*(mat(0,2)-mat(2,0));
//    z = 1/4/w*(mat(1,0)-mat(0,1));
//  } else {
//    x = sqrt(1+mat(0,0)-mat(1,1)-mat(2,2))/2;
//    y = 1/4/x*(mat(0,1)+mat(1,0));
//    z = 1/4/x*(mat(0,2)+mat(2,0));
//    w = 1/4/x*(mat(2,1)-mat(1,2));
//  }
//  q.w() = w;
//  q.x() = x;
//  q.y() = y;
//  q.z() = z;
//  q.normalize();

  return Eigen::Quaternion<TReturn>(A_IB.template cast<TReturn>());
}

template<typename T, typename TReturn = T>
static Eigen::Quaternion<TReturn> getQuaternionFromRotationMatrix(const Eigen::Matrix<T,3,3>& R_BI)
{
//  // Untested
//  // Bad precision!
//  double w;
//  double x;
//  double y;
//  double z;
//  w = sqrt(1+mat(0,0)+mat(1,1)+mat(2,2))/2;
//  if(w>0.02){
//    x = 1/4/w*(mat(2,1)-mat(1,2));
//    y = 1/4/w*(mat(0,2)-mat(2,0));
//    z = 1/4/w*(mat(1,0)-mat(0,1));
//  } else {
//    x = sqrt(1+mat(0,0)-mat(1,1)-mat(2,2))/2;
//    y = 1/4/x*(mat(0,1)+mat(1,0));
//    z = 1/4/x*(mat(0,2)+mat(2,0));
//    w = 1/4/x*(mat(2,1)-mat(1,2));
//  }
//  q.w() = w;
//  q.x() = x;
//  q.y() = y;
//  q.z() = z;
//  q.normalize();

  return Eigen::Quaternion<TReturn>(R_BI.template cast<TReturn>());
}

template<typename T, typename TReturn = T>
static Eigen::Quaternion<TReturn> getQuaternionFromRPY(const Eigen::Matrix<T,3,1>& rpy_BI)
{
//  // Tested and Working
//  Eigen::Quaternion<T> p_BI;
//
//  const T sr = sin(rpy_BI(0)/2);
//  const T cr = cos(rpy_BI(0)/2);
//  const T sp = sin(rpy_BI(1)/2);
//  const T cp = cos(rpy_BI(1)/2);
//  const T sy = sin(rpy_BI(2)/2);
//  const T cy = cos(rpy_BI(2)/2);
//
//  const T srsp = sr*sp;
//  const T srcp = sr*cp;
//  const T crsp = cr*sp;
//  const T crcp = cr*cp;
//
//  p_BI.w() = -srsp*sy+crcp*cy;
//  p_BI.x() = crsp*sy+srcp*cy;
//  p_BI.y() = crsp*cy-srcp*sy;
//  p_BI.z() = srsp*cy+crcp*sy;
////  p_BI.normalize();
//
//  return p_BI;

  return Eigen::Quaternion<TReturn>(
    Eigen::AngleAxis<TReturn>((TReturn)rpy_BI(0), Eigen::Matrix<TReturn, 3, 1>::UnitX()) *
    Eigen::AngleAxis<TReturn>((TReturn)rpy_BI(1), Eigen::Matrix<TReturn, 3, 1>::UnitY()) *
    Eigen::AngleAxis<TReturn>((TReturn)rpy_BI(2), Eigen::Matrix<TReturn, 3, 1>::UnitZ()));
}

template<typename T, typename TReturn = T>
static Eigen::Quaternion<TReturn> getQuaternionFromYPR(const Eigen::Matrix<T,3,1>& ypr_BI)
{
//  // Tested and Working
//  Eigen::Quaternion<T> p_BI;
//
//  const T sy = sin(ypr_BI(0)/2);
//  const T cy = cos(ypr_BI(0)/2);
//  const T sp = sin(ypr_BI(1)/2);
//  const T cp = cos(ypr_BI(1)/2);
//  const T sr = sin(ypr_BI(2)/2);
//  const T cr = cos(ypr_BI(2)/2);
//
//  const T sysp = sy*sp;
//  const T sycp = sy*cp;
//  const T cysp = cy*sp;
//  const T cycp = cy*cp;
//
//  p_BI.w() = sysp*sr+cycp*cr;
//  p_BI.x() = -sysp*cr+cycp*sr;
//  p_BI.y() = sycp*sr+cysp*cr;
//  p_BI.z() = sycp*cr-cysp*sr;
////  p_BI.normalize();
//
//  return p_BI;

  return Eigen::Quaternion<TReturn>(
    Eigen::AngleAxis<TReturn>((TReturn)ypr_BI(0), Eigen::Matrix<TReturn, 3, 1>::UnitZ()) *
    Eigen::AngleAxis<TReturn>((TReturn)ypr_BI(1), Eigen::Matrix<TReturn, 3, 1>::UnitY()) *
    Eigen::AngleAxis<TReturn>((TReturn)ypr_BI(2), Eigen::Matrix<TReturn, 3, 1>::UnitX()));
}


// 3) Output: Transformation Matrix

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getTransformationMatrixFromAngleAxis(const Eigen::AngleAxis<T>& aa_BI)
{
  return (aa_BI.template cast<TReturn>()).toRotationMatrix(); // A_IB
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getTransformationMatrixFromQuaternion(const Eigen::Quaternion<T>& p_BI)
{
  return (p_BI.template cast<TReturn>()).toRotationMatrix(); // A_IB
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getTransformationMatrixFromRotationMatrix(const Eigen::Matrix<T,3,3>& R_BI)
{
  return R_BI.template cast<TReturn>(); // A_IB
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getTransformationMatrixFromRPY(const Eigen::Matrix<T,3,1>& rpy_BI)
{
  Eigen::Matrix<TReturn,3,3> A_IB;

  const TReturn sr = sin(rpy_BI(0));
  const TReturn cr = cos(rpy_BI(0));
  const TReturn sp = sin(rpy_BI(1));
  const TReturn cp = cos(rpy_BI(1));
  const TReturn sy = sin(rpy_BI(2));
  const TReturn cy = cos(rpy_BI(2));

  const TReturn srsy = sr*sy;
  const TReturn srcy = sr*cy;
  const TReturn crsy = cr*sy;
  const TReturn crcy = cr*cy;

  A_IB(0,0) = cp*cy;
  A_IB(0,1) = -cp*sy;
  A_IB(0,2) = sp;
  A_IB(1,0) = crsy+srcy*sp;
  A_IB(1,1) = crcy-srsy*sp;
  A_IB(1,2) = -sr*cp;
  A_IB(2,0) = srsy-crcy*sp;
  A_IB(2,1) = srcy+crsy*sp;
  A_IB(2,2) = cr*cp;

  return A_IB;
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getTransformationMatrixFromYPR(const Eigen::Matrix<T,3,1>& ypr_BI)
{
  Eigen::Matrix<TReturn,3,3> A_IB;

  const TReturn sy = sin(ypr_BI(0));
  const TReturn cy = cos(ypr_BI(0));
  const TReturn sp = sin(ypr_BI(1));
  const TReturn cp = cos(ypr_BI(1));
  const TReturn sr = sin(ypr_BI(2));
  const TReturn cr = cos(ypr_BI(2));

  const TReturn sysr = sy*sr;
  const TReturn sycr = sy*cr;
  const TReturn cysr = cy*sr;
  const TReturn cycr = cy*cr;

  A_IB(0,0) = cy*cp;
  A_IB(0,1) = cysr*sp-sycr;
  A_IB(0,2) = sysr+cycr*sp;
  A_IB(1,0) = cp*sy;
  A_IB(1,1) = sysr*sp+cycr;
  A_IB(1,2) = sycr*sp-cysr;
  A_IB(2,0) = -sp;
  A_IB(2,1) = cp*sr;
  A_IB(2,2) = cp*cr;

  return A_IB;
}


// 4) Output: Rotation Matrix

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getRotationMatrixFromAngleAxis(const Eigen::AngleAxis<T>& aa_BI)
{
  return (aa_BI.template cast<TReturn>()).toRotationMatrix();
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getRotationMatrixFromQuaternion(const Eigen::Quaternion<T>& p_BI)
{
  return (p_BI.template cast<TReturn>()).toRotationMatrix();
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getRotationMatrixFromTransformationMatrix(const Eigen::Matrix<T,3,3>& A_IB)
{
  return A_IB.template cast<TReturn>(); // R_BI
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getRotationMatrixFromRPY(const Eigen::Matrix<T,3,1>& rpy_BI)
{
  Eigen::Matrix<TReturn,3,3> R_BI;

  const TReturn sr = sin(rpy_BI(0));
  const TReturn cr = cos(rpy_BI(0));
  const TReturn sp = sin(rpy_BI(1));
  const TReturn cp = cos(rpy_BI(1));
  const TReturn sy = sin(rpy_BI(2));
  const TReturn cy = cos(rpy_BI(2));

  const TReturn srsy = sr*sy;
  const TReturn srcy = sr*cy;
  const TReturn crsy = cr*sy;
  const TReturn crcy = cr*cy;

  R_BI(0,0) = cp*cy;
  R_BI(0,1) = -cp*sy;
  R_BI(0,2) = sp;
  R_BI(1,0) = crsy+srcy*sp;
  R_BI(1,1) = crcy-srsy*sp;
  R_BI(1,2) = -sr*cp;
  R_BI(2,0) = srsy-crcy*sp;
  R_BI(2,1) = srcy+crsy*sp;
  R_BI(2,2) = cr*cp;

  return R_BI;
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,3> getRotationMatrixFromYPR(const Eigen::Matrix<T,3,1>& ypr_BI)
{
  Eigen::Matrix<TReturn,3,3> R_BI;

  const TReturn sy = sin(ypr_BI(0));
  const TReturn cy = cos(ypr_BI(0));
  const TReturn sp = sin(ypr_BI(1));
  const TReturn cp = cos(ypr_BI(1));
  const TReturn sr = sin(ypr_BI(2));
  const TReturn cr = cos(ypr_BI(2));

  const TReturn sysr = sy*sr;
  const TReturn sycr = sy*cr;
  const TReturn cysr = cy*sr;
  const TReturn cycr = cy*cr;

  R_BI(0,0) = cy*cp;
  R_BI(0,1) = cysr*sp-sycr;
  R_BI(0,2) = sysr+cycr*sp;
  R_BI(1,0) = cp*sy;
  R_BI(1,1) = sysr*sp+cycr;
  R_BI(1,2) = sycr*sp-cysr;
  R_BI(2,0) = -sp;
  R_BI(2,1) = cp*sr;
  R_BI(2,2) = cp*cr;

  return R_BI;
}


// 5) Output: Roll-Pitch-Yaw

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getRPYFromAngleAxis(const Eigen::AngleAxis<T>& aa_BI)
{
  return (aa_BI.toRotationMatrix().eulerAngles(0, 1, 2)).template cast<TReturn>();
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getRPYFromQuaternion(const Eigen::Quaternion<T>& p_BI)
{
//  Eigen::Matrix<T,3,1> rpy_BI;
//
//  const T w = p_BI.w();
//  const T x = p_BI.x();
//  const T y = p_BI.y();
//  const T z = p_BI.z();
//
////  rpy_BI(1) = -asin(2*(x*z-w*y));
////
//////  std::cout << 2*(x*z-w*y)+1 << std::endl;
////
////  if(cos(rpy_BI(1)) == 0) // roll and yaw axis are the same -> yaw angle can be set to zero, roll does the whole rotation
////  {
////    rpy_BI(0) = -asin(2*(y*z-w*x));
////    rpy_BI(2) = 0;
////  }
////  else
////  {
////    rpy_BI(0) = atan2(2*(y*z+w*x),1-2*(x*x+y*y));
////    rpy_BI(2) = atan2(2*(x*y+w*z),1-2*(y*y+z*z));
////  }
//
//
//  // NEW
//
////  const T rpy_BI(1) = atan2(-2*(x*z-w*y),sqrt((1-2*(y*y+z*z))*(1-2*(y*y+z*z))+4*(x*y+w*z)*(x*y+w*z)));
//  const T test = 2*(x*z-w*y);
//
//  if(test > 0.999999999) // roll and yaw axis are the same -> yaw angle can be set to zero, roll does the whole rotation
//  {
////    rpy_BI(0) = -asin(2*(y*z-w*x));
////    rpy_BI(0) = acos(1-2*(x*x+z*z));
////    rpy_BI(0) = atan2(2*(y*z-w*x),1-2*(x*x+z*z));
//    rpy_BI(0) = 2*atan2(x,w);
//    rpy_BI(1) = -M_PI/2; // if test is a slight bit larger than 1, asin(test) does not work anymore
//    rpy_BI(2) = 0;
//  }
//  else if(test < -0.999999999)
//  {
////    rpy_BI(0) = -asin(2*(y*z-w*x));
////    rpy_BI(0) = acos(1-2*(x*x+z*z));
////    rpy_BI(0) = atan2(2*(y*z-w*x),1-2*(x*x+z*z));
//    rpy_BI(0) = -2*atan2(x,w);
//    rpy_BI(1) = M_PI/2;
//    rpy_BI(2) = 0;
//  }
//  else
//  {
//    rpy_BI(0) = atan2(2*(y*z+w*x),1-2*(x*x+y*y));
//    rpy_BI(1) = -asin(test);
//    rpy_BI(2) = atan2(2*(x*y+w*z),1-2*(y*y+z*z));
//  }
//
//  return rpy_BI;

  return p_BI.toRotationMatrix().eulerAngles(0, 1, 2);
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getRPYFromTransformationMatrix(const Eigen::Matrix<T,3,3>& A_IB)
{
  return (A_IB.eulerAngles(0, 1, 2)).template cast<TReturn>(); // rpy_BI
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getRPYFromRotationMatrix(const Eigen::Matrix<T,3,3>& R_BI)
{
  return (R_BI.eulerAngles(0, 1, 2)).template cast<TReturn>(); // rpy_BI
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getRPYFromYPR(const Eigen::Matrix<T,3,1>& ypr_BI)
{
  return getRPYFromQuaternion(getQuaternionFromYPR(ypr_BI.template cast<TReturn>()));
//  return getRPYFromAngleAxis(getAngleAxisFromYPR(ypr_BI));
}


// 6) Output: Yaw-Pitch-Roll

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getYPRFromAngleAxis(const Eigen::AngleAxis<T>& aa_BI)
{
  return (aa_BI.toRotationMatrix().eulerAngles(2, 1, 0)).template cast<TReturn>();
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getYPRFromQuaternion(const Eigen::Quaternion<T>& p_BI)
{
//  Eigen::Matrix<T,3,1> ypr_BI;
//
//  const T w = p_BI.w();
//  const T x = p_BI.x();
//  const T y = p_BI.y();
//  const T z = p_BI.z();
//
////  ypr_BI(1) = asin(2*(x*z+w*y));
////
////  if(cos(ypr_BI(1)) == 0) // yaw and roll axis are the same -> roll angle can be set to zero, yaw does the whole rotation
////  {
////    ypr_BI(0) = asin(2*(y*z+w*x));
////    ypr_BI(2) = 0;
////  }
////  else
////  {
////    ypr_BI(0) = -atan2(2*(x*y-w*z),1-2*(y*y+z*z));
////    ypr_BI(2) = -atan2(2*(y*z-w*x),1-2*(x*x+y*y));
////  }
//
//
//  // NEW
//
////  const T rpy_BI(1) = atan2(-2*(x*z-w*y),sqrt((1-2*(y*y+z*z))*(1-2*(y*y+z*z))+4*(x*y+w*z)*(x*y+w*z)));
//  const T test = 2*(x*z+w*y);
//
//  if(test > 0.999999999) // roll and yaw axis are the same -> yaw angle can be set to zero, roll does the whole rotation
//  {
////    ypr_BI(0) = asin(2*(y*z+w*x));
//    ypr_BI(0) = 2*atan2(x,w);
//    ypr_BI(1) = M_PI/2; // if test is a slight bit larger than 1, asin(test) does not work anymore
//    ypr_BI(2) = 0;
//  }
//  else if(test < -0.999999999)
//  {
////    ypr_BI(0) = -asin(2*(y*z+w*x));
//    ypr_BI(0) = -2*atan2(x,w);
//    ypr_BI(1) = -M_PI/2;
//    ypr_BI(2) = 0;
//  }
//  else
//  {
//    ypr_BI(0) = -atan2(2*(x*y-w*z),1-2*(y*y+z*z));
//    ypr_BI(1) = asin(test);
//    ypr_BI(2) = -atan2(2*(y*z-w*x),1-2*(x*x+y*y));
//  }
//
//  return ypr_BI;

  return (p_BI.toRotationMatrix().eulerAngles(2, 1, 0)).template cast<TReturn>();
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getYPRFromTransformationMatrix(const Eigen::Matrix<T,3,3>& A_IB)
{
  return (A_IB.eulerAngles(2, 1, 0)).template cast<TReturn>(); // ypr_BI
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getYPRFromRotationMatrix(const Eigen::Matrix<T,3,3>& R_BI)
{
  return (R_BI.eulerAngles(2, 1, 0)).template cast<TReturn>(); // ypr_BI
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getYPRFromRPY(const Eigen::Matrix<T,3,1>& rpy_BI)
{
  return getYPRFromQuaternion(getQuaternionFromRPY(rpy_BI.template cast<TReturn>()));
//  return getYPRFromAngleAxis(getAngleAxisFromRPY(rpy_BI));
}


// TODO inverse will be deleted in future

// 7) Output: Inverses

//template<typename T, typename TReturn = T>
//static Eigen::AngleAxis<TReturn> getInverseAngleAxis(const Eigen::AngleAxis<T>& aa_BI)
//{
//  return (aa_BI.template cast<TReturn>()).inverse();
//}
//
template<typename T, typename TReturn = T>
static Eigen::Quaternion<TReturn> getInverseQuaternion(const Eigen::Quaternion<T>& p_BI)
{
  return (p_BI.conjugate()).template cast<TReturn>();
}
//
//template<typename T, typename TReturn = T>
//static Eigen::Matrix<TReturn,3,3> getInverseTransformationMatrix(const Eigen::Matrix<T,3,3>& A_IB)
//{
//  return (A_IB.template cast<TReturn>()).transpose();
//}
//
//template<typename T, typename TReturn = T>
//static Eigen::Matrix<TReturn,3,3> getInverseRotationMatrix(const Eigen::Matrix<T,3,3>& R_BI)
//{
//  return (R_BI.template cast<TReturn>()).transpose();
//}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getInverseRPY(const Eigen::Matrix<T,3,1>& rpy_BI)
{
  return getRPYFromQuaternion(getInverseQuaternion(getQuaternionFromRPY(rpy_BI.template cast<TReturn>())));
}

template<typename T, typename TReturn = T>
static Eigen::Matrix<TReturn,3,1> getInverseYPR(const Eigen::Matrix<T,3,1>& ypr_BI)
{
  return getYPRFromQuaternion(getInverseQuaternion(getQuaternionFromYPR(ypr_BI.template cast<TReturn>())));
}




/*



Matrix3x4d get_H_bar(const Vector4d &p)
{
  Matrix3x4d H_bar = Matrix3x4d::Zero();
  const double w = p(0);
  const Vector3d e = p.tail(3);
  const Matrix3x3d eye3 = Matrix3x3d::Identity();

  H_bar.col(0) = -e;
  H_bar.block<3,3>(0,1) = -skewsymm(e)+w*eye3;

  return H_bar;
}


MatrixDyn get_F(const VectorDyn &q)
{
  const double dim_q = q.rows();
  const double dim_u = dim_q - 1;

  MatrixDyn F = MatrixDyn::Zero(dim_q,dim_u);
  const Vector4d p = q.block<4,1>(3,0);

  F.block<3,3>(0,0) = Matrix3x3d::Identity();
  F.block<4,3>(3,3) = 0.5*get_H_bar(p).transpose();
  if(dim_u > 6)
  {
    F.block(7,6,dim_u-6,dim_u-6) = MatrixDyn::Identity(dim_u-6,dim_u-6);
  }

  return F;
}

VectorDyn get_dqdt(const VectorDyn &q, const VectorDyn &u)
{
  const double dim_q = q.rows();
  const double dim_u = dim_q - 1;

  VectorDyn dqdt = VectorDyn::Zero(dim_q);
  const Vector4d p = q.block<4,1>(3,0);

  dqdt.head(3) = u.head(3);
  dqdt.block<4,1>(3,0) = 0.5*get_H_bar(p).transpose()*u.block<3,1>(3,0);
  if(dim_u > 6)
  {
    dqdt.tail(dim_u-6) = u.tail(dim_u-6);
  }

  return dqdt;
}

VectorDyn get_dqdt_2(const VectorDyn &q, const VectorDyn &u) // older version, slower
{
  return get_F(q)*u;
}

void prox1D(double &y, const double &x, const double &min, const double &max)
{
  if     (x < min) {y = min;}
  else if(x > max) {y = max;}
  else             {y = x;  }
}

void prox2D(double &y1, double &y2, const double &x1, const double &x2, const double &max)
{
  const double r = sqrt(x1*x1 + x2*x2);
  if(r > max)
  {
    y1 = max*x1/r;
    y2 = max*x2/r;
  }
  else
  {
    y1 = x1;
    y2 = x2;
  }
}

*/

/*


Vector4d multiplyQuaternion(const Vector4d &p_CB, const Vector4d &p_BA) // same in eigen
{
  // p_CA = p_CB*p_BA
  Vector4d p_CA = Vector4d::Zero();

  p_CA(0) = p_CB(0)*p_BA(0) - p_CB.tail(3).transpose()*p_BA.tail(3);
  p_CA.tail(3) = p_CB(0)*p_BA.tail(3) + p_BA(0)*p_CB.tail(3) + skewsymm(p_BA.tail(3))*p_CB.tail(3);

  p_CA.normalize();

  return p_CA;
}




double w_to_angleVel(const Vector3d &K_w_JK, const Vector3d &n)
{
//  if(n(0) == 1) // around x
//  {
//    return K_w_JK(0);
//  }
//  else
//  if(n(1) == 1) // around y
//  {
//    return K_w_JK(1);
//  }
//  else
//  if(n(2) == 1) // around z
//  {
//    return K_w_JK(2);
//  }

  const Vector3d n_norm = n.normalized();

  return n_norm.dot(K_w_JK); // attention: dot product doesn't eliminate small errors
}




Vector3d omega2kardan(const Vector3d &K_w_IK, const Vector3d &abc) // quaternion: rotation from I to K, kardan: x-y-z with alpha-beta-gamma
{
  double alpha = abc(0);
  double beta = abc(1);
  double gamma = abc(2);

  Vector3d dadbdc = Vector3d::Zero();
  Matrix3x3d H = Matrix3x3d::Zero();

  if(cos(beta) == 0)
  {
    H << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0, sin(gamma), cos(gamma), 0, -cos(gamma)*tan(beta), sin(gamma)*tan(beta), 1;
  }
  else
  {
    H << cos(gamma)/cos(beta), -sin(gamma)/cos(beta), 0, sin(gamma), cos(gamma), 0, -cos(gamma)*tan(beta), sin(gamma)*tan(beta), 1;
  }

  dadbdc = H*K_w_IK;

  return dadbdc;

  */





} // namespace rotations
} // namespace rm

#endif /* RM_ROTATION_EIGEN_FUNCTIONS_HPP_ */
