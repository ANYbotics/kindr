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

#ifndef KINDR_ROSGEOMETRYMSGPOSEEIGEN_HPP_
#define KINDR_ROSGEOMETRYMSGPOSEEIGEN_HPP_

#include "kindr/poses/PoseEigen.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include "kindr/thirdparty/ros/RosGeometryMsgPhysicalQuantitiesEigen.hpp"
#include "kindr/thirdparty/ros/RosGeometryMsgRotationEigen.hpp"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

namespace kindr {
namespace poses {
namespace eigen_impl {

template<typename PrimType_, typename Position_, typename Rotation_>
static void convertFromRosGeometryMsg(const geometry_msgs::Pose& geometryPoseMsg,
                                      HomogeneousTransformation<PrimType_, Position_, Rotation_>& pose)
{
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(geometryPoseMsg.position, pose.getPosition());

  // This is the definition of ROS Geometry pose.
  typedef rotations::eigen_impl::RotationQuaternion<double, rotations::RotationUsage::PASSIVE> RotationQuaternionGeometryPoseLike;

  RotationQuaternionGeometryPoseLike rotation;
  kindr::rotations::eigen_impl::convertFromRosGeometryMsg(geometryPoseMsg.orientation, rotation);
  pose.getRotation() = rotation;
}

template<typename PrimType_, typename Position_, typename Rotation_>
static void convertToRosGeometryMsg(const HomogeneousTransformation<PrimType_, Position_, Rotation_>& pose,
                                    geometry_msgs::Pose& geometryPoseMsg)
{
  kindr::phys_quant::eigen_impl::convertToRosGeometryMsg(pose.getPosition(), geometryPoseMsg.position);

  // This is the definition of ROS Geometry pose.
  typedef rotations::eigen_impl::RotationQuaternion<double, rotations::RotationUsage::PASSIVE> RotationQuaternionGeometryPoseLike;

  RotationQuaternionGeometryPoseLike rotation(pose.getRotation());
  kindr::rotations::eigen_impl::convertToRosGeometryMsg(rotation, geometryPoseMsg.orientation);
}

template<typename PrimType_, typename Position_, typename Rotation_>
static void convertFromRosGeometryMsg(const geometry_msgs::Transform& geometryTransformMsg,
                                      HomogeneousTransformation<PrimType_, Position_, Rotation_>& transformation)
{
  kindr::phys_quant::eigen_impl::convertFromRosGeometryMsg(geometryTransformMsg.translation, transformation.getPosition());

  // This is the definition of ROS Geometry transform.
  typedef rotations::eigen_impl::RotationQuaternion<double, rotations::RotationUsage::PASSIVE> RotationQuaternionGeometryTransformLike;

  RotationQuaternionGeometryTransformLike rotation;
  kindr::rotations::eigen_impl::convertFromRosGeometryMsg(geometryTransformMsg.rotation, rotation);
  transformation.getRotation() = rotation;
}

template<typename PrimType_, typename Position_, typename Rotation_>
static void convertToRosGeometryMsg(const HomogeneousTransformation<PrimType_, Position_, Rotation_>& transformation,
                                    geometry_msgs::Transform& geometryTransformMsg)
{
  kindr::phys_quant::eigen_impl::convertToRosGeometryMsg(transformation.getPosition(), geometryTransformMsg.translation);

  // This is the definition of ROS Geometry transform.
  typedef rotations::eigen_impl::RotationQuaternion<double, rotations::RotationUsage::PASSIVE> RotationQuaternionGeometryTransformLike;

  RotationQuaternionGeometryTransformLike rotation(transformation.getRotation());
  kindr::rotations::eigen_impl::convertToRosGeometryMsg(rotation, geometryTransformMsg.rotation);
}

} // namespace eigen_impl
} // namespace poses
} // namespace kindr

#endif /* KINDR_ROSGEOMETRYMSGPOSEEIGEN_HPP_ */
