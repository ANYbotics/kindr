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

#ifndef KINDR_ROSGEOMETRYMSGSEIGEN_HPP_
#define KINDR_ROSGEOMETRYMSGSEIGEN_HPP_

#include "kindr/phys_quant/PhysicalQuantitiesEigen.hpp"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

namespace kindr {
namespace phys_quant {
namespace eigen_impl {

template<typename PrimType_>
static void convertFromRosGeometryMsg(const geometry_msgs::Vector3& geometryVector3Msg,
                                      VectorTypeless<PrimType_, 3>& typelessVector)
{
  typelessVector.x() = static_cast<double>(geometryVector3Msg.x);
  typelessVector.y() = static_cast<double>(geometryVector3Msg.y);
  typelessVector.z() = static_cast<double>(geometryVector3Msg.z);
}

template<typename PrimType_>
static void convertToRosGeometryMsg(const VectorTypeless<PrimType_, 3>& typelessVector,
                                    geometry_msgs::Vector3& geometryVector3Msg)
{
  geometryVector3Msg.x = static_cast<double>(typelessVector.x());
  geometryVector3Msg.y = static_cast<double>(typelessVector.y());
  geometryVector3Msg.z = static_cast<double>(typelessVector.z());
}

static void convertFromRosGeometryMsg(const geometry_msgs::Point32& geometryPoint32Msg,
                                      Position3F& position)
{
  position.x() = geometryPoint32Msg.x;
  position.y() = geometryPoint32Msg.y;
  position.z() = geometryPoint32Msg.z;
}

static void convertToRosGeometryMsg(const Position3F& position,
                                    geometry_msgs::Point32& geometryPoint32Msg)
{
  geometryPoint32Msg.x = position.x();
  geometryPoint32Msg.y = position.y();
  geometryPoint32Msg.z = position.z();
}

static void convertFromRosGeometryMsg(const geometry_msgs::Point& geometryPoint32Msg,
                                      Position3D& position)
{
  position.x() = geometryPoint32Msg.x;
  position.y() = geometryPoint32Msg.y;
  position.z() = geometryPoint32Msg.z;
}

static void convertToRosGeometryMsg(const Position3D& position,
                                    geometry_msgs::Point& geometryPoint32Msg)
{
  geometryPoint32Msg.x = position.x();
  geometryPoint32Msg.y = position.y();
  geometryPoint32Msg.z = position.z();
}

} // namespace phys_quant
} // namespace eigen_impl
} // namespace kindr

#endif /* KINDR_ROSGEOMETRYMSGSEIGEN_HPP_ */
