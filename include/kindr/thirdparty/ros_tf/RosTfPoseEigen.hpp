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

#ifndef KINDR_ROSTFPOSEEIGEN_HPP_
#define KINDR_ROSTFPOSEEIGEN_HPP_

#include "kindr/poses/PoseEigen.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include "kindr/rotations/RotationBase.hpp"

// ROS
#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>

namespace kindr {
namespace poses {
namespace eigen_impl {

template<typename PrimType_, typename Position_, typename Rotation_>
static void convertFromRosTf(const tf::Transform& tfTransform, HomogeneousTransformation<PrimType_, Position_, Rotation_>& pose)
{
  typedef HomogeneousTransformation<PrimType_, Position_, Rotation_> Pose;
  typedef Position_ Position;
  typedef Rotation_ Rotation;

  rotations::eigen_impl::RotationMatrix<PrimType_, rotations::RotationUsage::PASSIVE> rotation;

  const tf::Vector3& rowX = tfTransform.getBasis().getRow(0);
  const tf::Vector3& rowY = tfTransform.getBasis().getRow(1);
  const tf::Vector3& rowZ = tfTransform.getBasis().getRow(2);

  rotation.setMatrix(rowX.x(), rowX.y(), rowX.z(),
                     rowY.x(), rowY.y(), rowY.z(),
                     rowZ.x(), rowZ.y(), rowZ.z());

  Position position(tfTransform.getOrigin().getX(),
                    tfTransform.getOrigin().getY(),
                    tfTransform.getOrigin().getZ());

  pose.getRotation() = Rotation(rotation);
  pose.getPosition() = position;
}


} // namespace eigen_impl
} // namespace poses
} // namespace kindr



#endif /* KINDR_ROSTFPOSEEIGEN_HPP_ */
