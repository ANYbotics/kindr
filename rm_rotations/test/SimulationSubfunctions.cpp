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

#include <iostream>

#include "SimulationSubfunctions.hpp"


Matrix3x3d skewsymm(const Vector3d &in)
{
	Matrix3x3d out;

	out << 0,-in(2),in(1),in(2),0,-in(0),-in(1),in(0),0;

	return out;
}

Matrix3x3d get_A_IB(const Vector4d &p_BI)
{
	const Vector4d p_BI_norm = p_BI.normalized();

	const double e0 = p_BI_norm(0);
	const Vector3d e = p_BI_norm.tail(3);
	const Matrix3x3d eye3 = Matrix3x3d::Identity();

	const Matrix3x3d A_IB = (2*e0*e0-1)*eye3 + 2*(e*e.transpose()+e0*skewsymm(e)); // transformation matrix from B to I

	return A_IB;
}

Matrix3x3d get_A_IB_2(const Vector4d &p_BI)
{
	const Vector4d p_BI_norm = p_BI.normalized();

	const double e0 = p_BI_norm(0);
	const double e1 = p_BI_norm(1);
	const double e2 = p_BI_norm(2);
	const double e3 = p_BI_norm(3);

	Matrix3x3d A_IB = Matrix3x3d::Zero();
	A_IB(0,0) = e0*e0 + e1*e1 - e2*e2 - e3*e3;
	A_IB(0,1) = 2*(e1*e2 - e0*e3);
	A_IB(0,2) = 2*(e0*e2 + e1*e3);
	A_IB(1,0) = 2*(e1*e2 + e0*e3);
	A_IB(1,1) = e0*e0 - e1*e1 + e2*e2 - e3*e3;
	A_IB(1,2) = 2*(e2*e3 - e0*e1);
	A_IB(2,0) = 2*(e1*e3 - e0*e2);
	A_IB(2,1) = 2*(e0*e1 + e2*e3);
	A_IB(2,2) = e0*e0 - e1*e1 - e2*e2 + e3*e3;

	return A_IB;
}

Matrix3x4d get_H_bar(const Vector4d &p)
{
	Matrix3x4d H_bar = Matrix3x4d::Zero();
	const double e0 = p(0);
	const Vector3d e = p.tail(3);
	const Matrix3x3d eye3 = Matrix3x3d::Identity();

	H_bar.col(0) = -e;
	H_bar.block<3,3>(0,1) = -skewsymm(e)+e0*eye3;

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

Vector4d createQuaternion(const Vector3d &n, const double &chi)
{
	Vector4d p_BA = Vector4d::Zero(); // quaternion describing the rotation from A to B
	Vector3d n_norm = n.normalized();

	p_BA << cos(chi/2),n_norm(0)*sin(chi/2),n_norm(1)*sin(chi/2),n_norm(2)*sin(chi/2);

	return p_BA;
}

Vector4d multiplyQuaternion(const Vector4d &p_CB, const Vector4d &p_BA)
{
	// p_CA = p_CB*p_BA
	Vector4d p_CA = Vector4d::Zero();

	p_CA(0) = p_CB(0)*p_BA(0) - p_CB.tail(3).transpose()*p_BA.tail(3);
	p_CA.tail(3) = p_CB(0)*p_BA.tail(3) + p_BA(0)*p_CB.tail(3) + skewsymm(p_BA.tail(3))*p_CB.tail(3);

	p_CA.normalize();

	return p_CA;
}

Vector4d A_to_Quaternion(const Matrix3x3d &A_IB)
{
	Vector4d p_BI = Vector4d::Zero();

	p_BI(0) = 0.5*sqrt(1.0+A_IB(0,0)+A_IB(1,1)+A_IB(2,2));

	if(p_BI(0) != 0)
	{
		p_BI(1) = 0.25*(A_IB(2,1)-A_IB(1,2))/p_BI(0);
		p_BI(2) = 0.25*(A_IB(0,2)-A_IB(2,0))/p_BI(0);
		p_BI(3) = 0.25*(A_IB(1,0)-A_IB(0,1))/p_BI(0);
	}
	else
	{
		std::cout << "error in SimulationSubfunctions.cpp" << std::endl;
		sleep(100);
	}

	return p_BI;
}

void Quaternion_to_AxisAngle(Vector3d &n, double &chi, const Vector4d &p_BI)
{
	n = p_BI.tail(3);

	if(n.norm() == 0)
	{
		n << 0,0,1;
		chi = 0;
	}
	else
	{
		n.normalize();

		if(n.sum() < 0)
		{
			n *= -1;
		}

		chi = 2*atan2(n.transpose()*p_BI.tail(3),p_BI(0));
	}


//	n = p_BI.tail(3);
//
//	if(n.norm() == 0)
//	{
//		n << 0,0,1;
//		chi = 0;
//	}
//	else
//	{
//		double n_max = std::max(std::max(abs(n(0)),abs(n(1))),abs(n(2)));
//
//		if(n_max == abs(n(0)))
//		{
//			n << 1,0,0;
//		}
//		else
//		if(n_max == abs(n(1)))
//		{
//			n << 0,1,0;
//		}
//		else
//		if(n_max == abs(n(2)))
//		{
//			n << 0,0,1;
//		}
//
//		chi = 2*atan2(n.transpose()*p_BI.tail(3),p_BI(0));
//	}
}

double A_to_angle(const Matrix3x3d &A_JK, const Vector3d &n)
{
	if(n(0) == 1) // around x
	{
		return atan2(A_JK(2,1),A_JK(2,2));
	}
	else
	if(n(1) == 1) // around y
	{
		return atan2(A_JK(0,2),A_JK(0,0));
	}
	else
	if(n(2) == 1) // around z
	{
		return atan2(A_JK(1,0),A_JK(1,1));
	}
}

double w_to_angleVel(const Vector3d &K_w_JK, const Vector3d &n)
{
//	if(n(0) == 1) // around x
//	{
//		return K_w_JK(0);
//	}
//	else
//	if(n(1) == 1) // around y
//	{
//		return K_w_JK(1);
//	}
//	else
//	if(n(2) == 1) // around z
//	{
//		return K_w_JK(2);
//	}

	const Vector3d n_norm = n.normalized();

	return n_norm.dot(K_w_JK); // attention: dot product doesn't eliminate small errors
}

Vector3d quat2euler(const Vector4d &p) // quaternion: rotation from I to K, euler: z-x-z with alpha-beta-gamma
{
	Vector4d p_norm = p.normalized();

	double e0 = p_norm(0);
	double e1 = p_norm(1);
	double e2 = p_norm(2);
	double e3 = p_norm(3);
	double alpha = 0;
	double beta = 0;
	double gamma = 0;

	Vector3d abc = Vector3d::Zero();

	beta = acos(2*(e0*e0+e3*e3)-1);

	if(sin(beta) == 0)
	{
		alpha = asin(2*e0*e3);
		gamma = 0;
	}
	else
	{
		alpha = atan2(e1*e3+e0*e2,-e2*e3+e0*e1);
		gamma = atan2(e1*e3-e0*e2,e2*e3+e0*e1);
	}

	abc << alpha, beta, gamma;

	return abc;
}

Vector3d omega2euler(const Vector3d &K_w_IK, const Vector3d &abc) // quaternion: rotation from I to K, euler: z-x-z with alpha-beta-gamma
{
	double alpha = abc(0);
	double beta = abc(1);
	double gamma = abc(2);

	Vector3d dadbdc = Vector3d::Zero();
	Matrix3x3d H = Matrix3x3d::Zero();

	if(sin(beta) == 0)
	{
		H << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0, cos(gamma), -sin(gamma), 0, 0, 0, 1;
	}
	else
	{
		H << sin(gamma)/sin(beta), cos(gamma)/sin(beta), 0, cos(gamma), -sin(gamma), 0, 0, 0, 1;
	}

	dadbdc = H*K_w_IK;

	return dadbdc;
}

Vector3d quat2kardan(const Vector4d &p) // quaternion: rotation from I to K, kardan: x-y-z with alpha-beta-gamma
{
	Vector4d p_norm = p.normalized();

	double e0 = p_norm(0);
	double e1 = p_norm(1);
	double e2 = p_norm(2);
	double e3 = p_norm(3);
	double alpha = 0;
	double beta = 0;
	double gamma = 0;

	Vector3d abc = Vector3d::Zero();

	beta = asin(2*(e1*e3+e0*e2));

	if(cos(beta) == 0)
	{
		alpha = asin(2*(e2*e3+e0*e1));
		gamma = 0;
	}
	else
	{
		alpha = - atan2(2*(e2*e3-e0*e1),2*(e0*e0+e3*e3)-1);
		gamma = - atan2(2*(e1*e2-e0*e3),2*(e0*e0+e1*e1)-1);
	}

	abc << alpha, beta, gamma;

	return abc;
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
}




