#include <gtest/gtest.h>
#include <rm/rotations/Rotations.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <random>
#include <sm/eigen/gtest.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <limits>
#include <sm/random.hpp>
#include <sm/timing/Timer.hpp>
#include "SimulationSubfunctions.hpp"
  #include <boost/math/special_functions/sign.hpp>
// use a timer
typedef sm::timing::Timer MyTimerType;
// use no timer
//typedef sm::timing::DummyTimer MyTimerType;

TEST (RotationsTest, testRotationMatrixFromKardanAngles	) {
	using namespace Eigen;

	// random seed
	sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));

	// create timers
	MyTimerType myTimer_A_CI_rot("A_CI_rot",true);
	MyTimerType myTimer_A_CI_proNEu("A_CI_proNEu",true);

	for (int i=0; i<10000; i++) {

		// rotation angles
		double alpha = 90/180.0*M_PI;	// roll
		double beta = 0;				// pitch
		double gamma = 0;				// yaw

		// random rotation angles
		alpha = sm::random::randLU(-2.0*M_PI,2.0*M_PI);
		beta = sm::random::randLU(-2.0*M_PI,2.0*M_PI);
		gamma = sm::random::randLU(-2.0*M_PI,2.0*M_PI);

		Matrix3d A_CB, A_BA, A_AI;
		A_CB << cos(gamma), sin(gamma), 0,
				-sin(gamma), cos(gamma), 0,
				0, 0, 1;

		A_BA << cos(beta), 0, -sin(beta),
				0, 1, 0,
				sin(beta), 0, cos(beta);

		A_AI << 1, 0, 0,
				0, cos(alpha), sin(alpha),
				0, -sin(alpha), cos(alpha);

		Matrix3d A_CI = A_CB*A_BA*A_AI;



		Matrix3d  A_IC_rot;
		A_IC_rot =	Quaterniond(AngleAxisd(alpha, Vector3d::UnitX()) *
				AngleAxisd(beta, Vector3d::UnitY()) *
				AngleAxisd(gamma, Vector3d::UnitZ()));


//		Matrix3d A_CI_rot;
//		A_CI_rot =	Quaterniond(AngleAxisd(-gamma, Vector3d::UnitZ()) *
//				AngleAxisd(-beta, Vector3d::UnitY()) *
//				AngleAxisd(-alpha, Vector3d::UnitX()));


		Quaterniond p_CI =	Quaterniond(AngleAxisd(-gamma, Vector3d::UnitZ()) *
				AngleAxisd(-beta, Vector3d::UnitY()) *
				AngleAxisd(-alpha, Vector3d::UnitX()));

		myTimer_A_CI_rot.start();
		Matrix3d A_CI_rot;
		A_CI_rot = p_CI;
		myTimer_A_CI_rot.stop();

		myTimer_A_CI_proNEu.start();
		Matrix3d A_CI_proNEu;
		const double t44 = cos(gamma);
		const double t45 = sin(alpha);
		const double t46 = sin(gamma);
		const double t47 = cos(alpha);
		const double t48 = sin(beta);
		const double t49 = cos(beta);
		A_CI_proNEu(0,0) = t44*t49;
		A_CI_proNEu(0,1) = t46*t47+t44*t45*t48;
		A_CI_proNEu(0,2) = t45*t46-t44*t47*t48;
		A_CI_proNEu(1,0) = -t46*t49;
		A_CI_proNEu(1,1) = t44*t47-t45*t46*t48;
		A_CI_proNEu(1,2) = t44*t45+t46*t47*t48;
		A_CI_proNEu(2,0) = t48;
		A_CI_proNEu(2,1) = -t45*t49;
		A_CI_proNEu(2,2) = t47*t49;
		myTimer_A_CI_proNEu.stop();


		ASSERT_DOUBLE_MX_EQ(A_CI, A_CI_rot, 1e-6, "A_CI_rot");
		ASSERT_DOUBLE_MX_EQ(A_CI, A_CI_proNEu, 1e-6, "A_CI_proNEu");
		ASSERT_DOUBLE_MX_EQ(A_CI, A_IC_rot.transpose(), 1e-6, "A_CI=A_IC_rot^T");
	}

	std::cout << sm::timing::Timing::print();

}

TEST (RotationsTest, DISABLED_testQuaternionFromRotationMatrix	) {
	using namespace Eigen;
	// random seed
	sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));

	// Kardan angles
	double alpha = 90/180.0*M_PI;	// roll
	double beta = 0;				// pitch
	double gamma = 0;				// yaw

	// random rotation angles
	alpha = sm::random::randLU(-2.0*M_PI,2.0*M_PI);
	beta = sm::random::randLU(-2.0*M_PI,2.0*M_PI);
	gamma = sm::random::randLU(-2.0*M_PI,2.0*M_PI);

	// quaternion from Kardan angles
	Quaterniond p_CI =	Quaterniond(AngleAxisd(-gamma, Vector3d::UnitZ()) *
				AngleAxisd(-beta, Vector3d::UnitY()) *
				AngleAxisd(-alpha, Vector3d::UnitX()));

	// roation matrix from quaternion
	Matrix3d A_CI;
	A_CI = p_CI.toRotationMatrix();

//	p_CI.normalize();
	p_CI = p_CI.inverse();

//	const double e0 = 0.5*sqrt(1.0+A_IC.trace());
//	const double e1 = -(A_IC(2,1)-A_IC(1,2))/(4.0*e0);
//	const double e2 = -(A_IC(0,2)-A_IC(2,0))/(4.0*e0);
//	const double e3 = -(A_IC(1,0)-A_IC(0,1))/(4.0*e0);

	Vector4d p_CI_remo = A_to_Quaternion(A_CI);
	const double e0 = p_CI_remo(0);
	const double e1 = p_CI_remo(1);
	const double e2 = p_CI_remo(2);
	const double e3 = p_CI_remo(3);
	EXPECT_NEAR(e0, p_CI.w(), 1e-6);
	EXPECT_NEAR(e1, p_CI.x(), 1e-6);
	EXPECT_NEAR(e2, p_CI.y(), 1e-6);
	EXPECT_NEAR(e3, p_CI.z(), 1e-6);

}

TEST (RotationsTest, testRemoQuatToRot) {
	using namespace Eigen;
	// random seed
	sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));

	Vector3d n = Vector3d(sm::random::randLU(-1000,1000), sm::random::randLU(-1000,1000), sm::random::randLU(-1000,1000));
	n.normalize();
	double chi = sm::random::randLU(-2*M_PI,2*M_PI);
	Vector4d p_CI = createQuaternion(n, chi);
	Vector3d n_new;
	double chi_new;

	Quaternion_to_AxisAngle(n_new, chi_new, p_CI);
	if (boost::math::sign(chi)==boost::math::sign(chi_new)) {
		EXPECT_NEAR(chi, chi_new, 1e-6);
		ASSERT_DOUBLE_MX_EQ(n, n_new, 1e-6, "n");
	} else {
		EXPECT_NEAR(chi, -chi_new, 1e-6);
		ASSERT_DOUBLE_MX_EQ(n, -n_new, 1e-6, "n");
	}



	Matrix3d A_IC = get_A_IB(p_CI);
	Vector4d p_CI_new = A_to_Quaternion(A_IC);
	if (boost::math::sign(p_CI(0))==boost::math::sign(p_CI_new(0))) {
		ASSERT_DOUBLE_MX_EQ(p_CI, p_CI_new, 1e-6, "n");
	} else {
		ASSERT_DOUBLE_MX_EQ(p_CI, -p_CI_new, 1e-6, "n");
	}

}



TEST (RotationsTest, testRotateVector) {
	using namespace Eigen;
	// random seed
	sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));

	// Kardan angles
	double alpha = 90/180.0*M_PI;	// roll
	double beta = 0;				// pitch
	double gamma = 0;				// yaw

	// random rotation angles
	alpha = sm::random::randLU(-2.0*M_PI,2.0*M_PI);
	beta = sm::random::randLU(-2.0*M_PI,2.0*M_PI);
	gamma = sm::random::randLU(-2.0*M_PI,2.0*M_PI);


	Matrix3d A_CI_p;
	const double t44 = cos(gamma);
	const double t45 = sin(alpha);
	const double t46 = sin(gamma);
	const double t47 = cos(alpha);
	const double t48 = sin(beta);
	const double t49 = cos(beta);
	A_CI_p(0,0) = t44*t49;
	A_CI_p(0,1) = t46*t47+t44*t45*t48;
	A_CI_p(0,2) = t45*t46-t44*t47*t48;
	A_CI_p(1,0) = -t46*t49;
	A_CI_p(1,1) = t44*t47-t45*t46*t48;
	A_CI_p(1,2) = t44*t45+t46*t47*t48;
	A_CI_p(2,0) = t48;
	A_CI_p(2,1) = -t45*t49;
	A_CI_p(2,2) = t47*t49;

	Vector3d I_r_OC = Vector3d(sm::random::randLU(-1000.0,1000.0), sm::random::randLU(-1000.0,1000.0), sm::random::randLU(-1000.0,1000.0));
	Vector3d C_r_OC_p = A_CI_p*I_r_OC;

	Quaterniond p_IC =	Quaterniond(AngleAxisd(alpha, Vector3d::UnitX()) *
				AngleAxisd(beta, Vector3d::UnitY()) *
				AngleAxisd(gamma, Vector3d::UnitZ()));


	Vector3d C_r_OC_eq, C_r_OC_eA;
	C_r_OC_eq = p_IC.inverse()*I_r_OC;
	ASSERT_DOUBLE_MX_EQ(C_r_OC_p, C_r_OC_eq, 1e-6, "C_r_OC_eq");

	C_r_OC_eA = p_IC.toRotationMatrix().transpose()*I_r_OC;
	ASSERT_DOUBLE_MX_EQ(C_r_OC_p, C_r_OC_eA, 1e-6, "C_r_OC_eA");


	Quaterniond p_CI =	Quaterniond( AngleAxisd(-gamma, Vector3d::UnitZ())*
				AngleAxisd(-beta, Vector3d::UnitY()) *
				AngleAxisd(-alpha, Vector3d::UnitX()));

	Vector3d C_r_OC_eq2;
	C_r_OC_eq2 = p_CI*I_r_OC;
	ASSERT_DOUBLE_MX_EQ(C_r_OC_p, C_r_OC_eq2, 1e-6, "C_r_OC_eq2");

	Vector3d C_r_OC_eA2;
	C_r_OC_eA2 = p_CI.toRotationMatrix()*I_r_OC;
	ASSERT_DOUBLE_MX_EQ(C_r_OC_p, C_r_OC_eA2, 1e-6, "C_r_OC_eA2");

}


//
//TEST (RotationsTest	, play) {
//	using namespace Eigen;
//
//
////	std::default_random_engine generator;
////	std::uniform_double_distribution<double> distribution;
//
////	   std::default_random_engine rng(std::random_device{}());
////	   std::uniform_real_distribution<double> dist(-100, 100);  //(min, max)
////
////	    //get one
////	    const double random_num = dist(rng);
//
//	double alpha = 90/180.0*M_PI;
//	double beta = 0;
//	double gamma = 0;
//
////	boost::random::mt19937 gen;
////	boost::random::uniform_real_distribution<> dist;
////	alpha = dist(gen);
//	sm::random::seed(static_cast<unsigned int>(std::time(nullptr)));
//	alpha = sm::random::randLU(-2.0*M_PI,2.0*M_PI);
//
//
//	std::cout << "alpha=" << alpha << std::endl;
////	alpha = distribution(generator);
//
//	Matrix3d A_CB, A_BA, A_AI;
//	A_CB << cos(gamma), sin(gamma), 0,
//			-sin(gamma), cos(gamma), 0,
//			0, 0, 1;
//
//	A_BA << cos(beta), 0, -sin(beta),
//			0, 1, 0,
//			sin(beta), 0, cos(beta);
//
//	A_AI << 1, 0, 0,
//			0, cos(alpha), sin(alpha),
//			0, -sin(alpha), cos(alpha);
//
//	Matrix3d A_CI = A_CB*A_BA*A_AI;
//
//
//	Matrix3d  A_IC_rot;
//	A_IC_rot =	Quaterniond(AngleAxisd(alpha, Vector3d::UnitX()) *
//			AngleAxisd(beta, Vector3d::UnitY()) *
//			AngleAxisd(gamma, Vector3d::UnitZ()));
//
//	Matrix3d A_CI_rot;
//
//	A_CI_rot =	Quaterniond(AngleAxisd(-alpha, Vector3d::UnitX()) *
//			AngleAxisd(-beta, Vector3d::UnitY()) *
//			AngleAxisd(-gamma, Vector3d::UnitZ()));
//
//	ASSERT_DOUBLE_MX_EQ(A_CI, A_CI_rot, 1e-9, "A_CI");
//	ASSERT_DOUBLE_MX_EQ(A_CI, A_IC_rot.transpose(), 1e-9, "A_CI=A_IC^T");
//
//
//	Quaterniond p(AngleAxisd(alpha, Vector3d::UnitY()));
//	Matrix3d A_IB = p.toRotationMatrix();
//	Vector3d I_r_OB = Vector3d(1.0, 2.0, 3.0);
//	Vector3d B_r_OB = A_IB.transpose()*I_r_OB;
//
//	Matrix3d A_IK = Quaterniond(AngleAxisd(90/180.0*M_PI, Vector3d::UnitX())).toRotationMatrix();
//	Vector3d K_r_OB = A_IK.transpose()*I_r_OB;
//
//	std::cout << "I_r_OB: " << I_r_OB.transpose() << std::endl;
//	std::cout << "B_r_OB: " << B_r_OB.transpose() << std::endl;
//	std::cout << "K_r_OB: " << K_r_OB.transpose() << std::endl;
////	Eigen::Matrix3d skewMatrix;
////	Eigen::Vector3d vec;
////	vec << 1, 2, 3;
////	skewMatrix << 0, -3, 2, 3, 0, -1, -2, 1, 0;
////	EXPECT_EQ(skewMatrix, rm::linear_algebra::getSkewMatrixFromVector(vec));
//}
