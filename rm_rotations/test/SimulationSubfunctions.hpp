/*
 * SimulationSubfunctions.hpp
 *
 *  Created on: Apr 9, 2013
 *      Author: aslteam
 */

#ifndef SIMULATIONSUBFUNCTIONS_HPP_
#define SIMULATIONSUBFUNCTIONS_HPP_

#include <Eigen/Core>


typedef Eigen::Matrix<double,19, 1> Vector19d;
typedef Eigen::Matrix<double,18, 1> Vector18d;
typedef Eigen::Matrix<double,12, 1> Vector12d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 1, 1> Vector1d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDyn;

typedef Eigen::Matrix<double, 7, 6> Matrix7x6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6x6d;
typedef Eigen::Matrix<double, 5, 5> Matrix5x5d;
typedef Eigen::Matrix<double, 4, 2> Matrix4x2d;
typedef Eigen::Matrix<double, 3,18> Matrix3x18d;
typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;
typedef Eigen::Matrix<double, 2, 3> Matrix2x3d;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDyn;

//asjdflk
Matrix3x3d skewsymm(const Vector3d &in); // asdf

Matrix3x3d get_A_IB(const Vector4d &p_BI);
Matrix3x3d get_A_IB_2(const Vector4d &p_BI);

Matrix3x4d get_H_bar(const Vector4d &p);
MatrixDyn get_F(const VectorDyn &q);
VectorDyn get_dqdt(const VectorDyn &q, const VectorDyn &u);
VectorDyn get_dqdt_2(const VectorDyn &q, const VectorDyn &u);

void prox1D(double &y, const double &x, const double &min, const double &max);
void prox2D(double &y1, double &y2, const double &x1, const double &x2, const double &max);

Vector4d createQuaternion(const Vector3d &n, const double &chi);
Vector4d multiplyQuaternion(const Vector4d &p_AB, const Vector4d &p_BC);

Vector4d A_to_Quaternion(const Matrix3x3d &A_IB);
void Quaternion_to_AxisAngle(Vector3d &n, double &chi, const Vector4d &p_BI);
double A_to_angle(const Matrix3x3d &A_JK, const Vector3d &n);
double w_to_angleVel(const Vector3d &K_w_JK, const Vector3d &n);

Vector3d quat2euler(const Vector4d &p);
Vector3d quat2kardan(const Vector4d &p);

Vector3d omega2euler(const Vector3d &K_w_IK, const Vector3d &abc);
Vector3d omega2kardan(const Vector3d &K_w_IK, const Vector3d &abc);





#endif /* SIMULATIONSUBFUNCTIONS_HPP_ */
