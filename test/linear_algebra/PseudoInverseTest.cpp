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
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/


#include <gtest/gtest.h>
#include <kindr/math/LinearAlgebra.hpp>
#include <kindr/common/gtest_eigen.hpp>

TEST (SquarePseudoInverseTest, testVector) {

	Eigen::Matrix2d A;
	A << 	61.6806, 345.8256,
				176.1109,  605.4883;

	Eigen::Matrix2d expectedPinvA;
	expectedPinvA << -0.0257, 0.0147,
										0.0075, -0.0026;

	Eigen::Matrix2d pinvA;
	kindr::pseudoInverse(A, pinvA);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(pinvA, expectedPinvA, 2e-3, 0, "");
}


TEST (RowPseudoInverseTest, testVector) {

	Eigen::Matrix<double, 3, 2> A;
	A << 0.3862, 0.2766,
			 0.3341, 0.0151,
			 0.4047, 0.3581;
	Eigen::MatrixXd Ad = A;

	Eigen::Matrix<double, 2, 3> expectedPinvA;
	expectedPinvA << 0.3864, 3.0637, -0.4272,
									 0.8652, -3.7645, 2.2824;
	Eigen::MatrixXd expectedPinvAd = expectedPinvA;

	Eigen::Matrix<double, 2, 3> pinvA;
	Eigen::MatrixXd pinvAd;

	kindr::pseudoInverse(A, pinvA);
	kindr::pseudoInverse(Ad, pinvAd);

	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(pinvA, expectedPinvA, 2e-3, 0, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(pinvAd, expectedPinvAd, 2e-3, 0, "");
}

TEST (ColPseudoInverseTest, testVector) {
	Eigen::Matrix<double, 2, 3> A;
	A << 0.6339, 0.6941, 0.6122,
			 0.7077, 0.3663, 0.1599;
	Eigen::MatrixXd Ad = A;

	Eigen::Matrix<double, 3, 2> expectedPinvA;
	expectedPinvA << -0.7783, 2.0147,
										0.8690, -0.4988,
										1.4542, -1.5207;
	Eigen::MatrixXd expectedPinvAd = expectedPinvA;

	Eigen::Matrix<double, 3, 2> pinvA;
	Eigen::MatrixXd pinvAd;

	kindr::pseudoInverse(A, pinvA);
	kindr::pseudoInverse(Ad, pinvAd);

  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(pinvA, expectedPinvA, 2e-3, 0, "");
	KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(pinvAd, expectedPinvAd, 2e-3, 0, "");
}