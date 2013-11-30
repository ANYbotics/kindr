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

#ifndef KINDER_POSES_POSESEIGEN_HPP_
#define KINDER_POSES_POSESEIGEN_HPP_

#include "kinder/common/common.hpp"
#include "kinder/common/assert_macros_eigen.hpp"
#include "kinder/positions/PositionEigen.hpp"
#include "kinder/rotations/RotationEigen.hpp"
#include "kinder/poses/PoseBase.hpp"


namespace kinder {
namespace poses {
//! Implementation of poses based on the C++ Eigen library
namespace eigen_implementation {


template<typename PrimType, typename POSITION, typename ROTATION>
class HomogeneousTransformation : public HomogeneousTransformationBase<HomogeneousTransformation<PrimType, POSITION, ROTATION>>, private POSITION, private ROTATION {
 public:

  typedef PrimType Scalar;
  typedef POSITION Position;
  typedef ROTATION Rotation;
  typedef Eigen::Matrix<PrimType, 4, 4> TransformationMatrix;


  HomogeneousTransformation() = default;

  HomogeneousTransformation(const Position& position, const Rotation& rotation) :
    Position(position),Rotation(rotation) {
  }


  inline POSITION & getPosition() {
    return static_cast<POSITION &>(*this);
  }

  inline const POSITION & getPosition() const {
    return static_cast<const POSITION &>(*this);
  }

  inline ROTATION & getRotation() {
    return static_cast<ROTATION &>(*this);
  }

  inline const ROTATION & getRotation() const {
    return static_cast<const ROTATION &>(*this);
  }


  inline TransformationMatrix getTransformationMatrix() const {
    TransformationMatrix mat = TransformationMatrix::Zero();
    mat.topLeftCorner(3,3) =  rotations::eigen_implementation::RotationMatrix<Scalar, kinder::rotations::RotationUsage::PASSIVE>(getRotation()).toImplementation();
    mat.topRightCorner(3,1) = getPosition().toImplementation();
    mat(3,3) = Scalar(1);
    return mat;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const HomogeneousTransformation & pose) {
    out << pose.getTransformationMatrix();
    return out;
  }


};

template<typename PrimType>
class HomogeneousTransformationPosition3RotationQuaternion: public HomogeneousTransformation<PrimType, positions::eigen_implementation::Position3<PrimType>, rotations::eigen_implementation::RotationQuaternion<PrimType, rotations::RotationUsage::PASSIVE>> {
 private:
  typedef HomogeneousTransformation<PrimType,kinder::positions::eigen_implementation::Position3<PrimType>, kinder::rotations::eigen_implementation::RotationQuaternion<PrimType, kinder::rotations::RotationUsage::PASSIVE>> Base;
 public:
  typedef PrimType Scalar;
  typedef typename Base::Position Position;
  typedef typename Base::Rotation Rotation;

  HomogeneousTransformationPosition3RotationQuaternion() = default;

  HomogeneousTransformationPosition3RotationQuaternion(const Position& position, const Rotation& rotation):
    Base(position, rotation) {
  }

};

typedef HomogeneousTransformationPosition3RotationQuaternion<double> HomogeneousTransformationPosition3RotationQuaternionD;
typedef HomogeneousTransformationPosition3RotationQuaternion<float> HomogeneousTransformationPosition3RotationQuaternionF;


} // namespace eigen_implementation
namespace internal {


template<typename PrimType, typename POSITION, typename ROTATION>
class get_position<eigen_implementation::HomogeneousTransformation<PrimType, POSITION, ROTATION>> {
 public:
  //! Position
  typedef POSITION Position;
};

template<typename PrimType, typename POSITION, typename ROTATION>
class TransformationTraits<eigen_implementation::HomogeneousTransformation<PrimType, POSITION, ROTATION>> {
 private:
  typedef typename eigen_implementation::HomogeneousTransformation<PrimType, POSITION, ROTATION> Pose;
  typedef typename get_position<Pose>::Position Position;
 public:
  inline static Position transform(const Pose & pose, const Position & position){
    return pose.getRotation().rotate(position) + pose.getPosition();
  }
  inline static Position inverseTransform(const Pose & pose, const Position & position){
    return pose.getRotation().inverseRotate((position-pose.getPosition()));
  }
};



} // namespace internal
} // namespace poses
} // namespace kinder

#endif /* KINDER_POSES_POSESEIGEN_HPP_ */
