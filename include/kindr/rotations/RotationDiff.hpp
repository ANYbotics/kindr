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

#pragma once

#include "kindr/math/LinearAlgebra.hpp"
#include "kindr/rotations/RotationBase.hpp"

namespace kindr {


template<typename PrimType_>
class LocalAngularVelocity;

template<typename PrimType_>
class GlobalAngularVelocity;

template<typename PrimType_>
class RotationQuaternionDiff;

template<typename PrimType_>
class RotationMatrixDiff;

template<typename PrimType_>
class EulerAnglesZyxDiff;

template<typename PrimType_>
class EulerAnglesXyzDiff;


/*!
 * \brief Gets the 3x3 Jacobian of the exponential map.
 * \param   vector 3x1-matrix
 * \return  matrix  (3x3-matrix)
 */
template<typename PrimType_>
inline static Eigen::Matrix<PrimType_, 3, 3> getJacobianOfExponentialMap(const Eigen::Matrix<PrimType_, 3, 1>& vector) {
  const PrimType_ norm = vector.norm();
  const Eigen::Matrix<PrimType_, 3, 3> skewMatrix = getSkewMatrixFromVector(vector);
  if (norm < 1.0e-4) {
    return Eigen::Matrix<PrimType_, 3, 3>::Identity() + 0.5*skewMatrix;
  }
  return Eigen::Matrix<PrimType_, 3, 3>::Identity() + (PrimType_(1.0) - cos(norm))/(norm*norm)*skewMatrix + (norm - sin(norm))/(norm*norm*norm)*(skewMatrix*skewMatrix);
}


} // namespace kindr


#include "kindr/rotations/LocalAngularVelocity.hpp"
#include "kindr/rotations/GlobalAngularVelocity.hpp"
#include "kindr/rotations/RotationQuaternionDiff.hpp"
#include "kindr/rotations/RotationMatrixDiff.hpp"
#include "kindr/rotations/EulerAnglesZyxDiff.hpp"
#include "kindr/rotations/EulerAnglesXyzDiff.hpp"


