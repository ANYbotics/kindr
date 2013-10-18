/*!
* @file 	Rotations.hpp
* @author 	Michael Blösch, Peter Fankhauser, Christian Gehring, Remo Diethelm
* @date		22 09, 2011
* @version 	1.0
* @ingroup 	rm
* @brief
*/
#ifndef RM_ROTATIONS_HPP_
#define RM_ROTATIONS_HPP_


#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rm {
namespace rotations {

// todo: quaternion: check norm with .squaredNorm()


// 1) Output: AngleAxis

template<typename T>
static Eigen::AngleAxis<T> getAngleAxisFromQuaternion(const Eigen::Quaternion<T> p_BI)
{
  return Eigen::AngleAxis<T>(p_BI);
}

template<typename T>
static Eigen::AngleAxis<T> getAngleAxisFromRotationMatrix(const Eigen::Matrix<T,3,3> R_BI)
{
  return Eigen::AngleAxis<T>(Eigen::Quaternion<T>(R_BI).conjugate());
//  return Eigen::AngleAxis<T>(R_BI); // does not work. why?
}

template<typename T>
static Eigen::AngleAxis<T> getAngleAxisFromRPY(const Eigen::Matrix<T,3,1> rpy_BI)
{
  return Eigen::AngleAxis<T>(
    Eigen::AngleAxis<T>(rpy_BI(0), Eigen::Matrix<T, 3, 1>::UnitX()) *
    Eigen::AngleAxis<T>(rpy_BI(1), Eigen::Matrix<T, 3, 1>::UnitY()) *
    Eigen::AngleAxis<T>(rpy_BI(2), Eigen::Matrix<T, 3, 1>::UnitZ()));
}

template<typename T>
static Eigen::AngleAxis<T> getAngleAxisFromYPR(const Eigen::Matrix<T,3,1> ypr_BI)
{
  return Eigen::AngleAxis<T>(
    Eigen::AngleAxis<T>(ypr_BI(0), Eigen::Matrix<T, 3, 1>::UnitZ()) *
    Eigen::AngleAxis<T>(ypr_BI(1), Eigen::Matrix<T, 3, 1>::UnitY()) *
    Eigen::AngleAxis<T>(ypr_BI(2), Eigen::Matrix<T, 3, 1>::UnitX()));
}


// 2) Output: Quaternion

template<typename T>
static Eigen::Quaternion<T> getQuaternionFromAngleAxis(const Eigen::AngleAxis<T> aa_BI)
{
  return Eigen::Quaternion<T>(aa_BI);
}

template<typename T>
static Eigen::Quaternion<T> getQuaternionFromRotationMatrix(const Eigen::Matrix<T,3,3> R_BI)
{
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

//  Quaternion<T> q_BI =

  return Eigen::Quaternion<T>(R_BI).conjugate();  // todo: what does this function?
}

template<typename T>
static Eigen::Quaternion<T> getQuaternionFromRPY(const Eigen::Matrix<T,3,1> rpy_BI)
{
//  Eigen::Quaternion<T> p_BI;
//
//  const T cy = cos(rpy_BI(2)/2);
//  const T sy = sin(rpy_BI(2)/2);
//  const T cc = cos(rpy_BI(1)/2)*cos(rpy_BI(0)/2);
//  const T cs = cos(rpy_BI(1)/2)*sin(rpy_BI(0)/2);
//  const T sc = sin(rpy_BI(1)/2)*cos(rpy_BI(0)/2);
//  const T ss = sin(rpy_BI(1)/2)*sin(rpy_BI(0)/2);
//
//  p_BI.w() = cy*cc-sy*ss;
//  p_BI.x() = -cy*cs-sy*sc;
//  p_BI.y() = -cy*sc+sy*cs;
//  p_BI.z() = -cy*ss-sy*cc;
//  p_BI.normalize();
//
//  return p_BI;

  return Eigen::Quaternion<T>(
    Eigen::AngleAxis<T>(rpy_BI(2), Eigen::Matrix<T, 3, 1>::UnitZ()) *
    Eigen::AngleAxis<T>(rpy_BI(1), Eigen::Matrix<T, 3, 1>::UnitY()) *
    Eigen::AngleAxis<T>(rpy_BI(0), Eigen::Matrix<T, 3, 1>::UnitX()));
}

template<typename T>
static Eigen::Quaternion<T> getQuaternionFromYPR(const Eigen::Matrix<T,3,1> ypr_BI)
{
  return Eigen::Quaternion<T>(
    Eigen::AngleAxis<T>(ypr_BI(2), Eigen::Matrix<T, 3, 1>::UnitX()) *
    Eigen::AngleAxis<T>(ypr_BI(1), Eigen::Matrix<T, 3, 1>::UnitY()) *
    Eigen::AngleAxis<T>(ypr_BI(0), Eigen::Matrix<T, 3, 1>::UnitZ()));
}


// 3) Output: Rotation Matrix (is equal to the transformation matrix in the Glocker notation)

template<typename T>
static Eigen::Matrix<T,3,3> getRotationMatrixFromAngleAxis(const Eigen::AngleAxis<T> aa_BI)
{
  return aa_BI.toRotationMatrix().transpose();
}

template<typename T>
static Eigen::Matrix<T,3,3> getRotationMatrixFromQuaternion(const Eigen::Quaternion<T> p_BI)
{
  return p_BI.toRotationMatrix().transpose();
}

template<typename T>
static Eigen::Matrix<T,3,3> getRotationMatrixFromRPY(const Eigen::Matrix<T,3,1> rpy_BI)
{
  Eigen::Matrix<T,3,3> R_BI;

  const T sr = sin(rpy_BI(0));
  const T cr = cos(rpy_BI(0));
  const T sp = sin(rpy_BI(1));
  const T cp = cos(rpy_BI(1));
  const T sy = sin(rpy_BI(2));
  const T cy = cos(rpy_BI(2));

  R_BI(0,0) = cy*cp;
  R_BI(0,1) = sy*cr+cy*sr*sp;
  R_BI(0,2) = sr*sy-cy*cr*sp;
  R_BI(1,0) = -sy*cp;
  R_BI(1,1) = cy*cr-sr*sy*sp;
  R_BI(1,2) = cy*sr+sy*cr*sp;
  R_BI(2,0) = sp;
  R_BI(2,1) = -sr*cp;
  R_BI(2,2) = cr*cp;

  return R_BI;
}

template<typename T>
static Eigen::Matrix<T,3,3> getRotationMatrixFromYPR(const Eigen::Matrix<T,3,1> ypr_BI)
{
  Eigen::Matrix<T,3,3> R_BI;

  const T sy = sin(ypr_BI(0));
  const T cy = cos(ypr_BI(0));
  const T sp = sin(ypr_BI(1));
  const T cp = cos(ypr_BI(1));
  const T sr = sin(ypr_BI(2));
  const T cr = cos(ypr_BI(2));

  R_BI(0,0) = cp*cy;
  R_BI(0,1) = cp*sy;
  R_BI(0,2) = -sp;
  R_BI(1,0) = sp*sr*cy-cr*sy;
  R_BI(1,1) = sp*sr*sy+cr*cy;
  R_BI(1,2) = cp*sr;
  R_BI(2,0) = sr*sy+sp*cr*cy;
  R_BI(2,1) = sp*cr*cy-sr*cy;
  R_BI(2,2) = cp*cr;

  return R_BI;
}


// 4) Output: Roll-Pitch-Yaw

template<typename T>
static Eigen::Matrix<T,3,1> getRPYFromAngleAxis(const Eigen::AngleAxis<T> aa_BI)
{
  return aa_BI.toRotationMatrix().eulerAngles(0, 1, 2);
}

template<typename T>
static Eigen::Matrix<T,3,1> getRPYFromQuaternion(const Eigen::Quaternion<T> p_BI)
{
  Eigen::Matrix<T,3,1> rpy_BI;

  const T w = p_BI.w();
  const T x = p_BI.x();
  const T y = p_BI.y();
  const T z = p_BI.z();

  rpy_BI(1) = -asin(2*(x*z-w*y));

  if(cos(rpy_BI(1)) == 0) // roll and yaw axis are the same -> yaw angle can be set to zero, roll does the whole rotation
  {
    rpy_BI(0) = -asin(2*(y*z-w*x));
    rpy_BI(2) = 0;
  }
  else
  {
    rpy_BI(0) = atan2(2*(y*z+w*x),1-2*(x*x+y*y));
    rpy_BI(2) = atan2(2*(x*y+w*z),1-2*(y*y+z*z));
  }

  return rpy_BI;

//  return p_BI.toRotationMatrix().eulerAngles(0, 1, 2);
}

template<typename T>
static Eigen::Matrix<T,3,1> getRPYFromRotationMatrix(const Eigen::Matrix<T,3,3> R_BI)
{
  return R_BI.transpose().eulerAngles(0, 1, 2);
}

template<typename T>
static Eigen::Matrix<T,3,1> getRPYFromYPR(const Eigen::Matrix<T,3,1> ypr_BI)
{
  return getRPYFromAngleAxis(getAngleAxisFromYPR(ypr_BI));
}


// 5) Output: Yaw-Pitch-Roll

template<typename T>
static Eigen::Matrix<T,3,1> getYPRFromAngleAxis(const Eigen::AngleAxis<T> aa_BI)
{
  return aa_BI.toRotationMatrix().eulerAngles(2, 1, 0);
}

template<typename T>
static Eigen::Matrix<T,3,1> getYPRFromQuaternion(const Eigen::Quaternion<T> p_BI)
{
  Eigen::Matrix<T,3,1> ypr_BI;

  const T w = p_BI.w();
  const T x = p_BI.x();
  const T y = p_BI.y();
  const T z = p_BI.z();

  ypr_BI(1) = asin(2*(x*z+w*y));

  if(cos(ypr_BI(1)) == 0) // yaw and roll axis are the same -> roll angle can be set to zero, yaw does the whole rotation
  {
    ypr_BI(0) = asin(2*(y*z+w*x));
    ypr_BI(2) = 0;
  }
  else
  {
    ypr_BI(0) = -atan2(2*(x*y-w*z),1-2*(y*y+z*z));
    ypr_BI(2) = -atan2(2*(y*z-w*x),1-2*(x*x+y*y));
  }

  return ypr_BI;

//  return p_BI.toRotationMatrix().eulerAngles(2, 1, 0);
}

template<typename T>
static Eigen::Matrix<T,3,1> getYPRFromRotationMatrix(const Eigen::Matrix<T,3,3> R_BI)
{
  return R_BI.transpose().eulerAngles(2, 1, 0);
}

template<typename T>
static Eigen::Matrix<T,3,1> getYPRFromRPY(const Eigen::Matrix<T,3,1> rpy_BI)
{
  return getYPRFromAngleAxis(getAngleAxisFromRPY(rpy_BI));
}


// 6) Output: Inverses

template<typename T>
static Eigen::AngleAxis<T> getInverseAngleAxis(const Eigen::AngleAxis<T> aa_BI)
{
  return aa_BI.inverse();
}

template<typename T>
static Eigen::Quaternion<T> getInverseQuaternion(const Eigen::Quaternion<T> p_BI)
{
  return p_BI.conjugate();
}

template<typename T>
static Eigen::Matrix<T,3,3> getInverseRotationMatrix(const Eigen::Matrix<T,3,3> R_BI)
{
  return R_BI.transpose();
}

template<typename T>
static Eigen::Matrix<T,3,1> getInverseRPY(const Eigen::Matrix<T,3,1> rpy_BI)
{
  return getRPYFromAngleAxis(getInverseAngleAxis(getAngleAxisFromRPY(rpy_BI)));
}

template<typename T>
static Eigen::Matrix<T,3,1> getInverseYPR(const Eigen::Matrix<T,3,1> ypr_BI)
{
  return getYPRFromAngleAxis(getInverseAngleAxis(getAngleAxisFromYPR(ypr_BI)));
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


Vector4d multiplyQuaternion(const Vector4d &p_CB, const Vector4d &p_BA)
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





} // end namespace rotations
} // end namespace rm

#endif /* RM_ROTATIONS_HPP_ */
