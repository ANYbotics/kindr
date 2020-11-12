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
#include "kindr/rotations/RotationDiff.hpp"
#include "kindr/poses/PoseDiffBase.hpp"

namespace kindr {

//! Twist, i.e. generalized 6D velocity in screw algebra.
template<typename PrimType_, typename PositionDiff_, typename RotationDiff_>
class Twist : public PoseDiffBase<Twist<PrimType_, PositionDiff_, RotationDiff_> >, private PositionDiff_, private RotationDiff_ {
 public:
  typedef PrimType_ Scalar;
  typedef PositionDiff_ PositionDiff;
  typedef RotationDiff_ RotationDiff;
  typedef Eigen::Matrix<PrimType_, 6, 1> Vector6;
  typedef Eigen::Matrix<PrimType_,3, 1> Vector3;

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
  inline Vector6 getVector(const RotationBase<Rotation_>& rotation) const {
    Vector6 vector = Vector6::Zero();

    vector.template block<3,1>(0,0) = getTranslationalVelocity().toImplementation();
    vector.template block<3,1>(3,0) =  LocalAngularVelocity<Scalar>(rotation.derived(), getRotationalVelocity()).toImplementation();
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

  /*! \brief Get zero twist
   *  \returns zero twist
   */
  static Twist Zero() {
    return Twist(PositionDiff::Zero(), RotationDiff::Zero());
  }
};

template<typename PrimType_>
class TwistLinearVelocityRotationQuaternionDiff: public Twist<PrimType_,
                                                              Velocity<PrimType_, 3>,
                                                              RotationQuaternionDiff<PrimType_>>
{
 private:
  typedef Twist<PrimType_, kindr::Velocity<PrimType_, 3>, RotationQuaternionDiff<PrimType_>> Base;
 public:
  typedef PrimType_ Scalar;
  typedef typename Base::PositionDiff PositionDiff;
  typedef typename Base::RotationDiff RotationDiff;

  TwistLinearVelocityRotationQuaternionDiff() = default;

  TwistLinearVelocityRotationQuaternionDiff(const PositionDiff& positionDiff, const RotationDiff& rotationDiff):
    Base(positionDiff, rotationDiff) {
  }

  /*! \brief Get zero twist
   *  \returns zero twist
   */
  static TwistLinearVelocityRotationQuaternionDiff Zero() {
    return TwistLinearVelocityRotationQuaternionDiff(PositionDiff::Zero(), RotationDiff::Zero());
  }
};

typedef TwistLinearVelocityRotationQuaternionDiff<double> TwistLinearVelocityRotationQuaternionDiffD;
typedef TwistLinearVelocityRotationQuaternionDiff<float> TwistLinearVelocityRotationQuaternionDiffF;

//! Body twist, i.e. twist where the angular velocity is expressed in the local body frame.
template<typename PrimType_>
class TwistLinearVelocityLocalAngularVelocity: public Twist<PrimType_,
                                                           Velocity<PrimType_, 3>,
                                                           LocalAngularVelocity<PrimType_>>
 {
 private:
  typedef Twist<PrimType_, kindr::Velocity<PrimType_, 3>, LocalAngularVelocity<PrimType_>> Base;
 public:
  typedef PrimType_ Scalar;
  typedef typename Base::PositionDiff PositionDiff;
  typedef typename Base::RotationDiff RotationDiff;
  typedef typename Base::Vector6 Vector6;
  typedef typename Base::Vector3 Vector3;

  TwistLinearVelocityLocalAngularVelocity() = default;

  TwistLinearVelocityLocalAngularVelocity(const PositionDiff& positionDiff, const RotationDiff& rotationDiff):
    Base(positionDiff, rotationDiff) {
  }

  TwistLinearVelocityLocalAngularVelocity(const Vector3& linearVelocity, const Vector3& localAngularVelocity):
    Base(PositionDiff(linearVelocity), RotationDiff(localAngularVelocity)) {
  }

  /*! Sets the twist by a 6D-vector [linear velocity, angular velocity]'
   */
  TwistLinearVelocityLocalAngularVelocity(const Vector6& vector6):
    Base(PositionDiff(vector6.template head<3>()), RotationDiff(vector6.template tail<3>())) {
  }

  /*!
   * @returns the twist in a 6D-vector [linear velocity, angular velocity]'
   */
  inline Vector6 getVector() const {
    Vector6 vector = Vector6::Zero();
    vector.template block<3,1>(0,0) = this->getTranslationalVelocity().toImplementation();
    vector.template block<3,1>(3,0) = this->getRotationalVelocity().toImplementation();
    return vector;
  }

  /*! Sets the twist by a 6D-vector [linear velocity, angular velocity]'
   */
  inline void setVector(const Vector6& vector6) {
    this->getTranslationalVelocity().toImplementation() = vector6.template head<3>();
    this->getRotationalVelocity().toImplementation() = vector6.template tail<3>();
  }

  /*! \brief Get zero twist
   *  \returns zero twist
   */
  static TwistLinearVelocityLocalAngularVelocity Zero() {
    return TwistLinearVelocityLocalAngularVelocity(PositionDiff::Zero(), RotationDiff::Zero());
  }
};

typedef TwistLinearVelocityLocalAngularVelocity<double> TwistLinearVelocityLocalAngularVelocityD;
typedef TwistLinearVelocityLocalAngularVelocity<float> TwistLinearVelocityLocalAngularVelocityF;
typedef TwistLinearVelocityLocalAngularVelocity<double> TwistLinearVelocityLocalAngularVelocityPD;
typedef TwistLinearVelocityLocalAngularVelocity<float> TwistLinearVelocityLocalAngularVelocityPF;
typedef TwistLinearVelocityLocalAngularVelocity<double> TwistLocalD;
typedef TwistLinearVelocityLocalAngularVelocity<float> TwistLocalF;

//! Spatial twist, i.e. twist where the angular velocity is expressed in the global parent frame.
template<typename PrimType_>
class TwistLinearVelocityGlobalAngularVelocity: public Twist<PrimType_,
                                                           Velocity<PrimType_, 3>,
                                                           GlobalAngularVelocity<PrimType_>>
 {
 private:
  typedef Twist<PrimType_, kindr::Velocity<PrimType_, 3>, GlobalAngularVelocity<PrimType_>> Base;
 public:
  typedef PrimType_ Scalar;
  typedef typename Base::PositionDiff PositionDiff;
  typedef typename Base::RotationDiff RotationDiff;
  typedef typename Base::Vector6 Vector6;
  typedef typename Base::Vector3 Vector3;

  TwistLinearVelocityGlobalAngularVelocity() = default;

  TwistLinearVelocityGlobalAngularVelocity(const PositionDiff& positionDiff, const RotationDiff& rotationDiff):
    Base(positionDiff, rotationDiff) {
  }

  TwistLinearVelocityGlobalAngularVelocity(const Vector3& linearVelocity, const Vector3& globalAngularVelocity):
    Base(PositionDiff(linearVelocity), RotationDiff(globalAngularVelocity)) {
  }

  /*! Sets the twist by a 6D-vector [linear velocity, angular velocity]'
   */
  TwistLinearVelocityGlobalAngularVelocity(const Vector6& vector6):
    Base(PositionDiff(vector6.template head<3>()), RotationDiff(vector6.template tail<3>())) {
  }

  /*!
   * @returns the twist in a 6D-vector [linear velocity, angular velocity]'
   */
  inline Vector6 getVector() const {
    Vector6 vector = Vector6::Zero();
    vector.template block<3,1>(0,0) = this->getTranslationalVelocity().toImplementation();
    vector.template block<3,1>(3,0) = this->getRotationalVelocity().toImplementation();
    return vector;
  }

  /*! Sets the twist by a 6D-vector [linear velocity, angular velocity]'
   */
  inline void setVector(const Vector6& vector6) {
    this->getTranslationalVelocity().toImplementation() = vector6.template head<3>();
    this->getRotationalVelocity().toImplementation() = vector6.template tail<3>();
  }

  /*! \brief Get zero twist
   *  \returns zero twist
   */
  static TwistLinearVelocityGlobalAngularVelocity Zero() {
    return TwistLinearVelocityGlobalAngularVelocity(PositionDiff::Zero(), RotationDiff::Zero());
  }
};

typedef TwistLinearVelocityGlobalAngularVelocity<double> TwistLinearVelocityGlobalAngularVelocityD;
typedef TwistLinearVelocityGlobalAngularVelocity<float> TwistLinearVelocityGlobalAngularVelocityF;
typedef TwistLinearVelocityGlobalAngularVelocity<double> TwistLinearVelocityGlobalAngularVelocityPD;
typedef TwistLinearVelocityGlobalAngularVelocity<float> TwistLinearVelocityGlobalAngularVelocityPF;
typedef TwistLinearVelocityGlobalAngularVelocity<double> TwistGlobalD;
typedef TwistLinearVelocityGlobalAngularVelocity<float> TwistGlobalF;

} // namespace kindr
