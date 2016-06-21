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

/*
 * convention.hpp
 *
 *  Created on: Jun 16, 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <kindr/Core>

namespace kindr {

/*! Implement the conversions from the kindr convention to the other convention.
 *  Specialize the conversion traits
 */
template<typename OtherRotation_, typename OtherVelocity_, typename PrimType_>
class RotationConversion {
 public:
  inline static void convertToOtherRotation(OtherRotation_& otherRotation, const kindr::RotationQuaternion<PrimType_>& quaternionIn) {
    // Implement rotation = f(quaternionIn);
    static_assert(sizeof(OtherRotation_) == -1, "You need to implement the method convertToOtherRotation!");
  }

  inline static void convertToKindr(kindr::RotationQuaternion<PrimType_>& quaternion, const OtherRotation_& otherRotation) {
    static_assert(sizeof(OtherRotation_) == -1, "You need to implement the method convertToKindr!");
  }

  inline static void convertToOtherVelocityVector(OtherVelocity_& otherVelocity, const OtherRotation_& rotation, const Eigen::Matrix<PrimType_,3,1>& velocityIn) {
    // Implement otherVelocity = g(velocityIn, rotation);
    static_assert(sizeof(OtherRotation_) == -1, "You need to implement the method convertToOtherVelocityVector!");
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix<PrimType_,3,3>& rotationMatrix, const OtherRotation_& rotation) {
    // Implement rotationMatrix = C(rotation);
    static_assert(sizeof(OtherRotation_) == -1, "You need to implement the method getRotationMatrixFromRotation!");
  }

  inline static void concatenate(OtherRotation_& result,  const OtherRotation_& rot1, OtherRotation_& rot2) {
    static_assert(sizeof(OtherRotation_) == -1, "You need to implement the method concatenate!");
  }

  inline static void rotateVector(Eigen::Matrix<PrimType_,3,1>& A_r, const OtherRotation_& rotationBToA, const Eigen::Matrix<PrimType_,3,1>& B_r) {
    // Implement A_r = rotationBtoA.rotate(A_r);
    static_assert(sizeof(OtherRotation_) == -1, "You need to implement the method rotateVector!");
  }

  inline static void boxPlus(OtherRotation_& result, const OtherRotation_& rotation, const OtherVelocity_& velocity) {
    // Implement result = rotation.boxPlus(vector);
    static_assert(sizeof(OtherRotation_) == -1, "You need to implement the method boxPlus!");
  }

  inline static bool testRotation(const OtherRotation_& expected, const OtherRotation_& actual) {
    // Implement EXPECT_NEAR(expected, actual, 1.0e-6);
    static_assert(sizeof(OtherRotation_) == -1, "You need to implement the method testRotation!");
    return false;
  }
};

} // namespace
