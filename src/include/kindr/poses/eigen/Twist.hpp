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

#ifndef KINDR_POSES_EIGEN_TWIST_HPP_
#define KINDR_POSES_EIGEN_TWIST_HPP_


#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/phys_quant/PhysicalQuantitiesEigen.hpp"
#include "kindr/rotations/RotationDiffEigen.hpp"
#include "kindr/poses/PoseDiffBase.hpp"

namespace kindr {
namespace poses {
//! Implementation of poses based on the C++ Eigen library
namespace eigen_impl {


template<typename PrimType_, typename PositionDiff_, typename RotationDiff_>
class Twist : public TwistBase<Twist<PrimType_, PositionDiff_, RotationDiff_>>, private PositionDiff_, private RotationDiff_ {
 public:

  typedef PrimType_ Scalar;
  typedef PositionDiff_ PositionDiff;
  typedef RotationDiff_ RotationDiff;
  typedef Eigen::Matrix<PrimType_, 6, 1> Vector6;


  Twist() = default;

  Twist(const PositionDiff& position, const RotationDiff& rotation) :
    PositionDiff(position),RotationDiff(rotation) {
  }


  inline PositionDiff_ & getTranslationalVelocity() {
    return static_cast<PositionDiff_ &>(*this);
  }

  inline const PositionDiff_ & getTranslationalVelocity() const {
    return static_cast<const PositionDiff_ &>(*this);
  }

  inline RotationDiff_ & getRotationalVelocity() {
    return static_cast<RotationDiff_ &>(*this);
  }

  inline const RotationDiff_ & getRotationalVelocity() const {
    return static_cast<const RotationDiff_ &>(*this);
  }

  template<typename Rotation_>
  inline Vector6 getVector(const rotations::RotationBase<Rotation_, rotations::RotationUsage::ACTIVE>& rotation) const {
    Vector6 vector = Vector6::Zero();

    vector.template block<3,1>(0,0) = getTranslationalVelocity().toImplementation();
    vector.template block<3,1>(3,0) =  rotations::eigen_impl::LocalAngularVelocity<Scalar, kindr::rotations::RotationUsage::ACTIVE>(rotation.derived(), getRotationalVelocity()).toImplementation();
    return vector;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const Twist & twist) {
    out << twist.getTranslationalVelocity() << " " << twist.getRotationalVelocity();
    return out;
  }

  /*! \brief Sets twist to zero
   *  \returns reference
   */
  Twist& setZero() {
    PositionDiff::setZero();
    RotationDiff::setZero();
    return *this;
  }
};

template<typename PrimType_>
class TwistLinearVelocityRotationQuaternionDiff: public Twist<PrimType_, kindr::phys_quant::eigen_impl::Velocity<PrimType_, 3>, rotations::eigen_impl::RotationQuaternionDiff<PrimType_, rotations::RotationUsage::ACTIVE>> {
 private:
  typedef Twist<PrimType_, kindr::phys_quant::eigen_impl::Velocity<PrimType_, 3>, kindr::rotations::eigen_impl::RotationQuaternionDiff<PrimType_, kindr::rotations::RotationUsage::ACTIVE>> Base;
 public:
  typedef PrimType_ Scalar;
  typedef typename Base::PositionDiff PositionDiff;
  typedef typename Base::RotationDiff RotationDiff;

  TwistLinearVelocityRotationQuaternionDiff() = default;

  TwistLinearVelocityRotationQuaternionDiff(const PositionDiff& positionDiff, const RotationDiff& rotationDiff):
    Base(positionDiff, rotationDiff) {
  }

};


typedef TwistLinearVelocityRotationQuaternionDiff<double> TwistLinearVelocityRotationQuaternionDiffD;
typedef TwistLinearVelocityRotationQuaternionDiff<float> TwistLinearVelocityRotationQuaternionDiffF;


template<typename PrimType_, enum rotations::RotationUsage Usage_>
class TwistLinearVelocityLocalAngularVelocity: public Twist<PrimType_, kindr::phys_quant::eigen_impl::Velocity<PrimType_, 3>, rotations::eigen_impl::LocalAngularVelocity<PrimType_, Usage_>> {
 private:
  typedef Twist<PrimType_, kindr::phys_quant::eigen_impl::Velocity<PrimType_, 3>, kindr::rotations::eigen_impl::LocalAngularVelocity<PrimType_, Usage_>> Base;
 public:
  typedef PrimType_ Scalar;
  typedef typename Base::PositionDiff PositionDiff;
  typedef typename Base::RotationDiff RotationDiff;
  typedef typename Base::Vector6 Vector6;

  TwistLinearVelocityLocalAngularVelocity() = default;

  TwistLinearVelocityLocalAngularVelocity(const PositionDiff& positionDiff, const RotationDiff& rotationDiff):
    Base(positionDiff, rotationDiff) {
  }


  inline Vector6 getVector() const {
    Vector6 vector = Vector6::Zero();
    vector.template block<3,1>(0,0) = this->getTranslationalVelocity().toImplementation();
    vector.template block<3,1>(3,0) = this->getRotationalVelocity().toImplementation();
    return vector;
  }

};

typedef TwistLinearVelocityLocalAngularVelocity<double, rotations::RotationUsage::ACTIVE> TwistLinearVelocityLocalAngularVelocityAD;
typedef TwistLinearVelocityLocalAngularVelocity<float, rotations::RotationUsage::ACTIVE> TwistLinearVelocityLocalAngularVelocityAF;
typedef TwistLinearVelocityLocalAngularVelocity<double, rotations::RotationUsage::PASSIVE> TwistLinearVelocityLocalAngularVelocityPD;
typedef TwistLinearVelocityLocalAngularVelocity<float, rotations::RotationUsage::PASSIVE> TwistLinearVelocityLocalAngularVelocityPF;

} // namespace eigen_impl

namespace internal {




} // namespace internal
} // namespace poses
} // namespace kindr

#endif /* KINDR_POSES_EIGEN_TWIST_HPP_ */
