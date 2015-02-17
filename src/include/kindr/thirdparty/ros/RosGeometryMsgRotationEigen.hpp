/*
 * Copyright (c) 2014, Peter Fankhauser, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
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
 * DISCLAIMED. IN NO EVENT SHALL Peter Fankhauser, Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef KINDR_ROSGEOMETRYMSGROTATIONEIGEN_HPP_
#define KINDR_ROSGEOMETRYMSGROTATIONEIGEN_HPP_

#include "kindr/rotations/RotationEigen.hpp"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

namespace kindr {
namespace rotations {
namespace eigen_impl {

template<typename PrimType_, enum kindr::rotations::RotationUsage Usage_>
static void convertFromRosGeometryMsg(const geometry_msgs::Quaternion& geometryQuaternionMsg,
                                      RotationQuaternion<PrimType_, Usage_>& rotationQuaternion)
{
  rotationQuaternion.setValues(static_cast<PrimType_>(geometryQuaternionMsg.w),
                               static_cast<PrimType_>(geometryQuaternionMsg.x),
                               static_cast<PrimType_>(geometryQuaternionMsg.y),
                               static_cast<PrimType_>(geometryQuaternionMsg.z));
}

template<typename PrimType_, enum kindr::rotations::RotationUsage Usage_>
static void convertToRosGeometryMsg(const RotationQuaternion<PrimType_, Usage_>& rotationQuaternion,
                                    geometry_msgs::Quaternion& geometryQuaternionMsg)
{
  geometryQuaternionMsg.w = static_cast<double>(rotationQuaternion.w());
  geometryQuaternionMsg.x = static_cast<double>(rotationQuaternion.x());
  geometryQuaternionMsg.y = static_cast<double>(rotationQuaternion.y());
  geometryQuaternionMsg.z = static_cast<double>(rotationQuaternion.z());
}

} // namespace eigen_impl
} // namespace rotations
} // namespace kindr

#endif /* KINDR_ROSGEOMETRYMSGROTATIONEIGEN_HPP_ */
