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

#include "kindr/poses/HomogeneousTransformation.hpp"

namespace kindr {

template<typename PrimType_, typename Position_, typename Rotation_>
class HomogeneousTransformation;


namespace internal {

/*! \brief Multiplication of two rotations with the same parameterization
 */
template<typename Left_, typename Right_>
class MultiplicationTraits<PoseBase<Left_>, PoseBase<Right_> > {
 public:
  typedef typename Left_::Position Position;
  typedef typename Left_::Rotation Rotation;
  typedef typename Left_::Scalar Scalar;
  typedef HomogeneousTransformation<Scalar, Position, Rotation> HomTrans;
 public:
  //! Default multiplication of rotations converts the representations of the rotations to rotation quaternions and multiplies them
  inline static Left_ mult(const PoseBase<Left_>& lhs, const PoseBase<Right_>& rhs) {
    const Position position = lhs.derived().getPosition()+lhs.derived().getRotation().rotate(rhs.derived().getPosition());
    const Rotation rotation = lhs.derived().getRotation()*rhs.derived().getRotation();
    return Left_(HomTrans(position, rotation));
  }
};

///*! \brief Multiplication of two rotations with the same parameterization
// */
//template<typename PrimType_, typename Position_, typename Rotation_>
//class MultiplicationTraits<HomogeneousTransformation<PrimType_, Position_, Rotation_>, HomogeneousTransformation<PrimType_, Position_, Rotation_> > {
// public:
//  //! Default multiplication of rotations converts the representations of the rotations to rotation quaternions and multiplies them
//  inline static HomogeneousTransformation<PrimType_, Position_, Rotation_> mult(const HomogeneousTransformation<PrimType_, Position_, Rotation_>& lhs, const HomogeneousTransformation<PrimType_, Position_, Rotation_>& rhs) {
//    const typename HomogeneousTransformation<PrimType_, Position_, Rotation_>::Position position = lhs.getPosition()+lhs.getRotation().rotate(rhs.getPosition());
//    const typename HomogeneousTransformation<PrimType_, Position_, Rotation_>::Rotation rotation = lhs.getRotation()*rhs.getRotation();
//    return HomogeneousTransformation<PrimType_, Position_, Rotation_>(position, rotation);
//  }
//};

///*! \brief Multiplication of two rotations with the same parameterization
// */
//template<typename LeftAndRight_>
//class MultiplicationTraits<RotationBase<LeftAndRight_>, RotationBase<LeftAndRight_> > {
// public:
//  //! Default multiplication of rotations converts the representations of the rotations to rotation quaternions and multiplies them
//  inline static LeftAndRight_ mult(const RotationBase<LeftAndRight_>& lhs, const RotationBase<LeftAndRight_>& rhs) {
//      return LeftAndRight_(RotationQuaternion<typename LeftAndRight_::Scalar>(
//                          (RotationQuaternion<typename LeftAndRight_::Scalar>(lhs.derived())).toImplementation() *
//                          (RotationQuaternion<typename LeftAndRight_::Scalar>(rhs.derived())).toImplementation()
//                          ));
//
//  }
//};

/*! \brief Compare two poses.
 */
template<typename Left_, typename Right_>
class ComparisonTraits<PoseBase<Left_>, PoseBase<Right_>>{
 public:
  inline static bool isEqual(const PoseBase<Left_>& lhs, const PoseBase<Right_>& rhs) {
    return lhs.derived().getPosition() == rhs.derived().getPosition() &&
           lhs.derived().getRotation() == rhs.derived().getRotation();
  }
};


} // namespace internal
} // namespace kindr
