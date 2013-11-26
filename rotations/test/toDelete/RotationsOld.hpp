/*!
* @file 	Rotations.hpp
* @author 	Michael Blösch, Peter Fankhauser, Christian Gehring
* @date		22 09, 2011
* @version 	1.0
* @ingroup 	rm
* @brief	Rotation stuff... convention:
* 			roll-pitch-yaw: 	alias
* 			rotation matrix: 	alibi
* 			rotation vector: 	alibi
* 			quaternion:			alibi
*/
#ifndef RM_ROTATIONS_HPP_
#define RM_ROTATIONS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rm {
namespace rotationsOld {


/*! Converts vector to sqew matrix
 * @param[in] 	vec		vector
 * @param[out] 	mat		sqew-matrix
 */
static void vecToSqew(const Eigen::Vector3d& vec, Eigen::Matrix3d& mat){
	mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
}
/*! Converts quaternions to
 * @param[in] 	q		quaternion
 * @param[out] 	rpy		roll-pitch-yaw
 */
static void quatToRpy(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy){
	const double q0 = q.w();
	const double q1 = q.x();
	const double q2 = q.y();
	const double q3 = q.z();
	rpy(0) = -atan2(2*(q0*q1+q2*q3),1-2*(pow(q1,2)+pow(q2,2)));
	rpy(1) = -asin(2*(q0*q2-q1*q3));
	rpy(2) = -atan2(2*(q0*q3+q1*q2),1-2*(pow(q2,2)+pow(q3,2)));
}

static void rpyToQuat(const Eigen::Vector3d& rpy, Eigen::Quaterniond& q){
	const double cy = cos(rpy(2)/2);
	const double sy = sin(rpy(2)/2);
	const double cc = cos(rpy(1)/2)*cos(rpy(0)/2);
	const double cs = cos(rpy(1)/2)*sin(rpy(0)/2);
	const double sc = sin(rpy(1)/2)*cos(rpy(0)/2);
	const double ss = sin(rpy(1)/2)*sin(rpy(0)/2);

	q.w() = cy*cc-sy*ss;
	q.x() = -cy*cs-sy*sc;
	q.y() = -cy*sc+sy*cs;
	q.z() = -cy*ss-sy*cc;
	q.normalize();
}

/*!
 * Converts yaw, pitch and roll angles to a quaternion.
* Based on the MIRA project (http://www.mira-project.org).
*
 * @param [in] yaw (first rotation around the Z-Axis (pointing upwards))
 * @param [in] pitch (second rotation around the Y-Axis (pointing left))
 * @param [in] roll (third rotation around the X-Axis (pointing forward))
 * @return quaternion
 */
template<typename T>
static Eigen::Quaternion<T> yawPitchRollToQuaternion(const T& yaw, const T& pitch, const T& roll)
{
return Eigen::Quaternion<T>(
	Eigen::AngleAxis<T>(yaw, Eigen::Matrix<T, 3, 1>::UnitZ()) *
	Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY()) *
	Eigen::AngleAxis<T>(roll, Eigen::Matrix<T, 3, 1>::UnitX()));
}

/*!
* Converts a quaternion back to yaw, pitch, roll angles.
*
* This is operation is the opposite of yawPitchRollToQuaternion().
* Please note: If a quaternion was created using quaternionFromYawPitchRoll()
* this method may return yaw, pitch, roll angles that may differ from the values
* used in the quaternionFromYawPitchRoll() call. The reason is that a single
* rotation can be described by different combinations of yaw, pitch and
* roll angles. For details see yawPitchRollToQuaternion().
*
* Based on the MIRA project (http://www.mira-project.org).
*
* @param [in] q the quaternion
* @return yaw, pitch and roll angle as vector with length 3
*/
template<typename T>
static Eigen::Matrix<T,3,1> quaternionToYawPitchRoll(const Eigen::Quaternion<T>& q)
{
  return q.toRotationMatrix().eulerAngles(2, 1, 0);
}

/*!
* Converts a angle-axis rotation to yaw, pitch, roll angles.
*
* @param [in] angleAxis the rotation in angle-axis representation
* @return yaw, pitch and roll angle as vector with length 3
*/
template<typename T>
static Eigen::Matrix<T,3,1> angleAxisToYawPitchRoll(const Eigen::AngleAxis<T>& angleAxis)
{
return angleAxis.toRotationMatrix().eulerAngles(2, 1, 0);
}

static void rpyToEar(const Eigen::Vector3d& rpy, Eigen::Matrix3d& ear){
	const double cp = cos(rpy(1));
	const double sp = sin(rpy(1));
	const double cy = cos(rpy(2));
	const double sy = sin(rpy(2));
	ear << cp*cy, sy, 0, -cp*sy, cy, 0, sp, 0, 1;
}

static void rpyToEarInv(const Eigen::Vector3d& rpy, Eigen::Matrix3d& earInv){
	const double t2 = cos(rpy(1));
	const double t3 = 1.0/t2;
	const double t4 = sin(rpy(2));
	const double t5 = cos(rpy(2));
	const double t6 = tan(rpy(1));
	earInv(0,0) = t3*t5;
	earInv(0,1) = -t3*t4;
	earInv(0,2) = 0;
	earInv(1,0) = t4;
	earInv(1,1) = t5;
	earInv(1,2) = 0;
	earInv(2,0) = -t5*t6;
	earInv(2,1) = t4*t6;
	earInv(2,2) = 1;
}

static void quatToRotMat(const Eigen::Quaterniond& q, Eigen::Matrix3d& mat){
	const double q0 = q.w();
	const double q1 = q.x();
	const double q2 = q.y();
	const double q3 = q.z();
	Eigen::Vector3d vec = Eigen::Vector3d(q1,q2,q3);
	vecToSqew(vec,mat);
	mat = (2*pow(q0,2)-1)*Eigen::Matrix3d::Identity() + 2*q0*mat + 2 * vec * vec.transpose();
}

static void quatToRotVec(const Eigen::Quaterniond& q, Eigen::Vector3d& vec){
	double cos = q.w();
	if(cos > 1.0){
		cos = 1;
	} else if(cos < -1.0){
		cos = -1.0;
	}
	const double angle = 2*acos(cos);
	vec(0,0) = q.x();
	vec(1,0) = q.y();
	vec(2,0) = q.z();
	double norm = vec.norm();
	if(norm >= 1e-20){
		vec = vec*angle/vec.norm();
	} else {
		vec.setZero();
	}
}

static void rotVecToQuat(const Eigen::Vector3d& vec, Eigen::Quaterniond& q){
	const double angle = vec.norm();
	double a;
	if(angle >= 1e-20){
		a = sin(angle/2.0)/angle;
	} else {
		a = 0;
	}
	q.w() = cos(angle/2.0);
	q.x() = vec(0)*a;
	q.y() = vec(1)*a;
	q.z() = vec(2)*a;
	q.normalize();
}

static void quatInverse(const Eigen::Quaterniond& q, Eigen::Quaterniond& q2){
	q2.w() = q.w();
	q2.x() = -q.x();
	q2.y() = -q.y();
	q2.z() = -q.z();
	q2.normalize();
}

static void quatLeftMultMat(const Eigen::Matrix<double,4,1>& q, Eigen::Matrix<double,4,4>& M){
	M.setIdentity();
	M = M*q(3);
	M(0,1) = -q(2);
	M(0,2) = q(1);
	M(1,0) = q(2);
	M(1,2) = -q(0);
	M(2,0) = -q(1);
	M(2,1) = q(0);
	M.block<1,4>(3,0) = -q;
	M.block<4,1>(0,3) = q;
}

static void quatRightMultMat(const Eigen::Matrix<double,4,1>& q, Eigen::Matrix<double,4,4>& M){
	M.setIdentity();
	M = M*q(3);
	M(0,1) = q(2);
	M(0,2) = -q(1);
	M(1,0) = -q(2);
	M(1,2) = q(0);
	M(2,0) = q(1);
	M(2,1) = -q(0);
	M.block<1,4>(3,0) = -q;
	M.block<4,1>(0,3) = q;
}


static void rotMatToQuat(const Eigen::Matrix3d& mat, Eigen::Quaterniond& q){
	// Bad precision!
	double w;
	double x;
	double y;
	double z;
	w = sqrt(1+mat(0,0)+mat(1,1)+mat(2,2))/2;
	if(w>0.02){
		x = 1/4/w*(mat(2,1)-mat(1,2));
		y = 1/4/w*(mat(0,2)-mat(2,0));
		z = 1/4/w*(mat(1,0)-mat(0,1));
	} else {
		x = sqrt(1+mat(0,0)-mat(1,1)-mat(2,2))/2;
		y = 1/4/x*(mat(0,1)+mat(1,0));
		z = 1/4/x*(mat(0,2)+mat(2,0));
		w = 1/4/x*(mat(2,1)-mat(1,2));
	}
	q.w() = w;
	q.x() = x;
	q.y() = y;
	q.z() = z;
	q.normalize();
}

static void rotMatToRotVec(const Eigen::Matrix3d& mat, Eigen::Vector3d& vec){
	double cosTheta = 0.5*(mat(0,0)+mat(1,1)+mat(2,2)-1);
	if(cosTheta > 1.0){
		cosTheta = 1;
	} else if(cosTheta < -1.0){
		cosTheta = -1.0;
	}
	double theta = acos(cosTheta);
	vec(0) = (mat(2,1)-mat(1,2));
	vec(1) = (mat(0,2)-mat(2,0));
	vec(2) = (mat(1,0)-mat(0,1));
	if(std::fabs(theta) > 1e-10){
		vec *= 0.5/sin(theta)*theta;
	} else {
		vec *= 0.5;
	}
}


static void rpyToRotMat(const Eigen::Vector3d& rpy, Eigen::Matrix3d& mat){
	const double t1 = cos(rpy(2));
	const double t2 = sin(rpy(0));
	const double t3 = sin(rpy(2));
	const double t4 = cos(rpy(0));
	const double t5 = sin(rpy(1));
	const double t6 = cos(rpy(1));
	mat(0,0) = t1*t6;
	mat(0,1) = t3*t4+t1*t2*t5;
	mat(0,2) = t2*t3-t1*t4*t5;
	mat(1,0) = -t3*t6;
	mat(1,1) = t1*t4-t2*t3*t5;
	mat(1,2) = t1*t2+t3*t4*t5;
	mat(2,0) = t5;
	mat(2,1) = -t2*t6;
	mat(2,2) = t4*t6;
}

static void reduceAngle(double& angle){
	angle = -2.0*M_PI*floor((angle+M_PI)/(2*M_PI))+angle;
}


} // end namespace rotations
} // end namespace rm

#endif /* RM_ROTATIONS_HPP_ */
