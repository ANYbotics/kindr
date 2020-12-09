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

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/phys_quant/PhysicalQuantities.hpp"
#include "kindr/rotations/Rotation.hpp"
#include "kindr/poses/PoseBase.hpp"


namespace kindr {


template<typename PrimType_, typename Position_, typename Rotation_>
class HomogeneousTransformation : public PoseBase<HomogeneousTransformation<PrimType_, Position_, Rotation_> > {
 protected:
  Position_ position_;
  Rotation_ rotation_;
 public:

  typedef PrimType_ Scalar;
  typedef Position_ Position;
  typedef Rotation_ Rotation;
  typedef Eigen::Matrix<PrimType_, 4, 4> TransformationMatrix;


  explicit HomogeneousTransformation(): position_(), rotation_() {

  }

  inline explicit HomogeneousTransformation(const Position& position, const Rotation& rotation) :
    position_(position),rotation_(rotation) {
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit HomogeneousTransformation(const PoseBase<OtherDerived_>& other)
    : position_(Position(other.derived().getPosition())), rotation_(Rotation(other.derived().getRotation()))
  {
    // todo: use conversion trait
  }

  /*! \brief Assignment operator
   *  \param other   other transformation
   *  \returns reference
   */
  HomogeneousTransformation& operator =(const HomogeneousTransformation& other) = default;

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

  /*! \brief Concenation operator.
   *  This is explicitly specified, because RotationBase provides also an operator*.
   *  \returns the concenation of two tansformations
   */
  using PoseBase<HomogeneousTransformation<PrimType_, Position_, Rotation_> >::operator*;

  inline TransformationMatrix getTransformationMatrix() const {
    TransformationMatrix mat = TransformationMatrix::Zero();
    mat.template topLeftCorner<3,3>() =  RotationMatrix<Scalar>(getRotation()).toImplementation();
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

  /*! \brief Sets the pose to identity
   *  \returns reference
   */
  HomogeneousTransformation& setIdentity() {
    position_.setZero();
    rotation_.setIdentity();
    return *this;
  }

  /*! \brief Get identity pose.
   *  \returns identity pose
   */
  static HomogeneousTransformation Identity() {
    return HomogeneousTransformation(Position::Zero(), Rotation::Identity());
  }
};

template <typename PrimType_>
using HomTransformQuat = HomogeneousTransformation<PrimType_, Position<PrimType_, 3>, RotationQuaternion<PrimType_>>;
typedef HomTransformQuat<double> HomTransformQuatD;
typedef HomTransformQuat<float> HomTransformQuatF;

// For backwards comp.
template <typename PrimType_>
using HomogeneousTransformationPosition3RotationQuaternion = HomogeneousTransformation<PrimType_, Position<PrimType_, 3>, RotationQuaternion<PrimType_>>;
typedef HomogeneousTransformationPosition3RotationQuaternion<double> HomogeneousTransformationPosition3RotationQuaternionD;
typedef HomogeneousTransformationPosition3RotationQuaternion<float> HomogeneousTransformationPosition3RotationQuaternionF;

template <typename PrimType_>
using HomTransformMatrix = HomogeneousTransformation<PrimType_, Position<PrimType_, 3>, RotationMatrix<PrimType_>>;
typedef HomTransformMatrix<double> HomTransformMatrixD;
typedef HomTransformMatrix<float> HomTransformMatrixF;


namespace internal {

template<typename PrimType_, typename Position_, typename Rotation_>
class get_position<HomogeneousTransformation<PrimType_, Position_, Rotation_>> {
 public:
  //! Position
  typedef Position_ Position;
};

template<typename PrimType_, typename Position_, typename Rotation_>
class TransformationTraits<HomogeneousTransformation<PrimType_, Position_, Rotation_>> {
 private:
  typedef HomogeneousTransformation<PrimType_, Position_, Rotation_> Pose;
  typedef typename get_position<Pose>::Position Translation;
 public:
  inline static Translation transform(const Pose & pose, const Translation & position){
    return pose.getRotation().rotate(position) + pose.getPosition();
  }
  inline static Translation inverseTransform(const Pose & pose, const Translation & position){
    return pose.getRotation().inverseRotate((position-pose.getPosition()));
  }
};


/*! \brief Multiplication of two rotations with the same parameterization
 */
template<typename PrimType_, typename Position_, typename Rotation_>
class MultiplicationTraits<PoseBase<HomogeneousTransformation<PrimType_, Position_, Rotation_>>, PoseBase<HomogeneousTransformation<PrimType_, Position_, Rotation_>> > {
 public:
  //! Default multiplication of rotations converts the representations of the rotations to rotation quaternions and multiplies them
  inline static HomogeneousTransformation<PrimType_, Position_, Rotation_> mult(const HomogeneousTransformation<PrimType_, Position_, Rotation_>& lhs, const HomogeneousTransformation<PrimType_, Position_, Rotation_>& rhs) {
    const typename HomogeneousTransformation<PrimType_, Position_, Rotation_>::Position position = lhs.getPosition()+lhs.getRotation().rotate(rhs.getPosition());
    const typename HomogeneousTransformation<PrimType_, Position_, Rotation_>::Rotation rotation = lhs.getRotation()*rhs.getRotation();
    return HomogeneousTransformation<PrimType_, Position_, Rotation_>(position, rotation);
  }
};



} // namespace internal
} // namespace kindr

