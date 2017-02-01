/*
 * Copyright (c) 2017, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
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
 * Wernch.hpp
 *
 *  Created on: Jan 31, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <kindr/phys_quant/WrenchBase.hpp>

namespace kindr {

//!
template <typename PrimType_, typename Force_, typename Torque_>
class Wrench : public WrenchBase<Wrench<PrimType_, Force_, Torque_>> {
public:
  typedef PrimType_ Scalar;
  typedef Force_ Force;
  typedef Torque_ Torque;
  typedef Eigen::Matrix<PrimType_, 6, 1> Vector6;
  typedef Eigen::Matrix<PrimType_,3, 1> Vector3;

  Wrench() : force_(Force::Zero()),  torque_(Torque::Zero()) {

  }

  Wrench(const Force& force, const Torque& torque) :
    force_(force),
    torque_(torque) {
  }

  inline Force & getForce() {
    return force_;
  }

  inline const Force & getForce() const {
    return force_;
  }

  inline Torque & getTorque() {
    return torque_;
  }

  inline const Torque & getTorque() const {
    return torque_;
  }

  inline Vector6 getVector() const {
    Vector6 vector;
    vector.template block<3,1>(0,0) = getForce().toImplementation();
    vector.template block<3,1>(3,0) = getTorque().toImplementation();
    return vector;
  }

  Wrench& setZero() {
    force_.setZero();
    torque_.setZero();
    return *this;
  }

protected:
  Force force_;
  Torque torque_;
};

typedef Wrench<double, kindr::Force3D, kindr::Torque3D> WrenchD;
typedef Wrench<float, kindr::Force3F, kindr::Torque3F> WrenchF;


} // namespace kindr
