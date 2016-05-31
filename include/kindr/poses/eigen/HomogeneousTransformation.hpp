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

#ifndef KINDR_POSES_EIGEN_HOMOGENEOUSTRANSFORMATION_HPP_
#define KINDR_POSES_EIGEN_HOMOGENEOUSTRANSFORMATION_HPP_

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/phys_quant/PhysicalQuantitiesEigen.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include "kindr/poses/PoseBase.hpp"


namespace kindr {
namespace poses {
//! Implementation of poses based on the C++ Eigen library
namespace eigen_impl {


template<typename PrimType_, typename Position_, typename Rotation_>
class HomogeneousTransformation : public TransformationBase<HomogeneousTransformation<PrimType_, Position_, Rotation_>> {
 public:

  typedef PrimType_ Scalar;
  typedef Position_ Position;
  typedef Rotation_ Rotation;
  typedef Eigen::Matrix<PrimType_, 4, 4> TransformationMatrix;
  
private:
  Position position_;
  Rotation rotation_;
public:

  HomogeneousTransformation() = default;

  HomogeneousTransformation(const Position& position, const Rotation& rotation) :
    position_(position), rotation_(rotation) {
  }


  inline Position_ & getPosition() {
    return position_;
  }

  inline const Position_ & getPosition() const {
    return position_;
  }

  inline Rotation_ & getRotation() {
    return rotation_;
  }

  inline const Rotation_ & getRotation() const {
    return rotation_;
  }


  inline TransformationMatrix getTransformationMatrix() const {
    TransformationMatrix mat = TransformationMatrix::Zero();
    mat.template topLeftCorner<3,3>() =  rotations::eigen_impl::RotationMatrix<Scalar, kindr::rotations::RotationUsage::PASSIVE>(getRotation()).toImplementation();
    mat.template topRightCorner<3,1>() = getPosition().toImplementation();
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

template<typename PrimType_>
class HomogeneousTransformationPosition3RotationQuaternion: public HomogeneousTransformation<PrimType_, phys_quant::eigen_impl::Position<PrimType_, 3>, rotations::eigen_impl::RotationQuaternion<PrimType_, rotations::RotationUsage::PASSIVE>> {
 private:
  typedef HomogeneousTransformation<PrimType_, phys_quant::eigen_impl::Position<PrimType_, 3>, kindr::rotations::eigen_impl::RotationQuaternion<PrimType_, kindr::rotations::RotationUsage::PASSIVE>> Base;
 public:
  typedef PrimType_ Scalar;
  typedef typename Base::Position Position;
  typedef typename Base::Rotation Rotation;

  HomogeneousTransformationPosition3RotationQuaternion() = default;

  HomogeneousTransformationPosition3RotationQuaternion(const Position& position, const Rotation& rotation):
    Base(position, rotation) {
  }

};

typedef HomogeneousTransformation<double, phys_quant::eigen_impl::Position<double, 3>, rotations::eigen_impl::RotationQuaternion<double, rotations::RotationUsage::PASSIVE>> HomogeneousTransformationPosition3RotationQuaternionD;

typedef HomogeneousTransformation<float, phys_quant::eigen_impl::Position<float, 3>, rotations::eigen_impl::RotationQuaternion<float, rotations::RotationUsage::PASSIVE>> HomogeneousTransformationPosition3RotationQuaternionF;


} // namespace eigen_impl
namespace internal {


template<typename PrimType_, typename Position_, typename Rotation_>
class get_position<eigen_impl::HomogeneousTransformation<PrimType_, Position_, Rotation_>> {
 public:
  //! Position
  typedef Position_ Position;
};

template<typename PrimType_, typename Position_, typename Rotation_>
class TransformationTraits<eigen_impl::HomogeneousTransformation<PrimType_, Position_, Rotation_>> {
 private:
  typedef typename eigen_impl::HomogeneousTransformation<PrimType_, Position_, Rotation_> Pose;
  typedef typename get_position<Pose>::Position Position;
 public:

};



} // namespace internal
} // namespace poses
} // namespace kindr


#endif /* KINDR_POSES_EIGEN_HOMOGENEOUSTRANSFORMATION_HPP_ */
