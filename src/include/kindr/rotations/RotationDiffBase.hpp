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

#ifndef KINDR_ROTATIONS_RDIFFBASE_HPP_
#define KINDR_ROTATIONS_RDIFFBASE_HPP_


#include "kindr/common/common.hpp"
#include "kindr/rotations/RotationBase.hpp"

namespace kindr {
namespace rotations {
//! Internal stuff (only for developers)
namespace internal {

/*! \brief Addition traits for angular velocities
 *  \class RotationDiffAdditionTraits
 *  (only for advanced users)
 */
template<typename Left_, typename Right_>
class RotationDiffAdditionTraits {
 public:
//  inline static LeftAndRight_ add(const LeftAndRight_& lhs, const LeftAndRight_& rhs);
//  inline static LeftAndRight_ subtract(const LeftAndRight_& lhs, const LeftAndRight_& rhs);
};

/*! \brief Conversion traits for converting derivatives of rotations into each other
 *  \class RotationDiffConversionTraits
 *  (only for advanced users)
 */
template<typename Dest_, typename Source_, typename Rotation_>
class RotationDiffConversionTraits {
 public:
  // inline static Dest_ convert(const Rotation_&, const Source_& );
};


} // namespace internal

/*! \class RotationDiffBase
 * \brief Interface for an angular velocity.
 *
 * This class defines the generic interface for an angular velocity.
 * More precisely an interface to store and access the angular velocity of a rigid body is provided.
 * \tparam Derived_ the derived class that should implement the angular velocity.
 * \ingroup rotations
 *
 */
template<typename Derived_, enum RotationUsage Usage_>
class RotationDiffBase {
 public:
  /*! \brief Rotation usage.
   *  The rotation usage is either active (rotation of an object) or passive (transformation of its coordinates)
   */
  static constexpr enum RotationUsage Usage = Usage_;

  /*! \brief Default constructor.
   *
    *  Creates a time derivative of a rotation with zero coefficients.
    */
  RotationDiffBase() = default;

  /*! \brief Constructor from derived time derivative of a rotation.
   *
   *  This constructor has been deleted because the abstract class does not contain any data.
   */
  RotationDiffBase(const Derived_&) = delete; // on purpose!!

  /*! \brief Gets the derived time derivative of a rotation.
   *  (only for advanced users)
   *  \returns the derived time derivative of a rotation
   */
  operator Derived_& () {
    return static_cast<Derived_&>(*this);
  }

  /*! \brief Gets the derived time derivative of a rotation.
   *  (only for advanced users)
   *  \returns the derived time derivative of a rotation
   */
  operator const Derived_& () const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief Gets the derived time derivative of a rotation.
   *  (only for advanced users)
   *  \returns the derived time derivative of a rotation
   */
  const Derived_& derived() const {
    return static_cast<const Derived_&>(*this);
  }


  /*! \brief Sets the time derivative of a rotation to zero.
   *  \returns reference
   */
  Derived_& setZero();



  /*! \brief Addition of two time derivatives.
   *  \returns sum of the two time derivatives.
   */
  template<typename OtherDerived_>
  Derived_ operator +(const RotationDiffBase<OtherDerived_, Usage_>& other) const {
    return internal::RotationDiffAdditionTraits<RotationDiffBase<Derived_, Usage_>, RotationDiffBase<OtherDerived_, Usage_>>::add(this->derived(), other.derived()); // todo: 1. ok? 2. may be optimized
  }

  /*! \brief Subtraction of two time derivatives.
   *  \returns result of the subtraction of the two time derivatives.
   */
  template<typename OtherDerived_>
  Derived_ operator -(const RotationDiffBase<OtherDerived_, Usage_>& other) const {
    return internal::RotationDiffAdditionTraits<RotationDiffBase<Derived_, Usage_>, RotationDiffBase<OtherDerived_, Usage_>>::subtract(this->derived(), other.derived()); // todo: 1. ok? 2. may be optimized
  }

  /*! \brief Addition and assignment.
   *  \returns sum of the two time derivatives.
   */
  template<typename OtherDerived_>
  Derived_& operator +=(const RotationDiffBase<OtherDerived_, Usage_>& other);

  /*! \brief Subtraction and assignment.
   *  \returns result of the subtraction of the two time derivatives.
   */
  template<typename OtherDerived_>
  Derived_& operator -=(const RotationDiffBase<OtherDerived_, Usage_>& other);

};

/*! \class AngularVelocityBase
 * \brief Interface for an angular velocity of a rigid body in 3D-space.
 *
 * This class defines the generic interface for an angular velocity in 3D-space.
 * More precisely an interface to store and access the three components of the angular velocity of a rigid body is provided.

 * \tparam Derived_ the derived class that should implement the angular velocity.
 *
 *  \ingroup rotations
 */
template<typename Implementation_, enum RotationUsage Usage_>
class AngularVelocityBase : public RotationDiffBase<Implementation_, Usage_> {


};


template<typename Implementation_, enum RotationUsage Usage_>
class LocalAngularVelocityBase : public AngularVelocityBase<Implementation_, Usage_> {


};

template<typename Implementation_, enum RotationUsage Usage_>
class AngleAxisDiffBase : public RotationDiffBase<Implementation_, Usage_> {
 public:

};

template<typename Implementation_, enum RotationUsage Usage_>
class RotationVectorDiffBase : public RotationDiffBase<Implementation_, Usage_> {
 public:

};

template<typename Implementation_, enum RotationUsage Usage_>
class RotationQuaternionDiffBase : public RotationDiffBase<Implementation_, Usage_> {
 public:

};

template<typename Implementation_, enum RotationUsage Usage_>
class RotationMatrixDiffBase : public RotationDiffBase<Implementation_, Usage_> {
 public:

};


template<typename Implementation_, enum RotationUsage Usage_>
class EulerAnglesDiffBase : public RotationDiffBase<Implementation_, Usage_> {
 public:

};


template<typename Implementation_, enum RotationUsage Usage_>
class EulerAnglesDiffZyxBase : public EulerAnglesDiffBase<Implementation_, Usage_> {
 public:

};

template<typename Implementation_, enum RotationUsage Usage_>
class EulerAnglesDiffXyzBase : public EulerAnglesDiffBase<Implementation_, Usage_> {
 public:

};



namespace internal {

template<typename Left_, typename Right_, enum RotationUsage Usage_>
class RotationDiffAdditionTraits<RotationDiffBase<Left_, Usage_>, RotationDiffBase<Right_, Usage_>> {
 public:
  inline static Left_ add(const RotationDiffBase<Left_, Usage_>& lhs, const RotationDiffBase<Right_, Usage_>& rhs) {
    return Left_(typename eigen_impl::LocalAngularVelocity<typename Left_::Scalar, Usage_>(
               (typename eigen_impl::LocalAngularVelocity<typename Left_::Scalar, Usage_>(lhs.derived())).toImplementation() +
               (typename eigen_impl::LocalAngularVelocity<typename Right_::Scalar, Usage_>(rhs.derived())).toImplementation()
               ));
  }
  inline static Left_ subtract(const RotationDiffBase<Left_, Usage_>& lhs, const RotationDiffBase<Right_, Usage_>& rhs) {
    return Left_(typename eigen_impl::LocalAngularVelocity<typename Left_::Scalar, Usage_>(
               (typename eigen_impl::LocalAngularVelocity<typename Left_::Scalar, Usage_>(lhs.derived())).toImplementation() -
               (typename eigen_impl::LocalAngularVelocity<typename Right_::Scalar, Usage_>(rhs.derived())).toImplementation()
               ));
  }
};


template<typename LeftAndRight_, enum RotationUsage Usage_>
class RotationDiffAdditionTraits<RotationDiffBase<LeftAndRight_, Usage_>, RotationDiffBase<LeftAndRight_, Usage_>> {
 public:
  /*! \returns the sum of two angular velocities
   * \param lhs left-hand side
   * \param rhs right-hand side
   */
  inline static LeftAndRight_ add(const RotationDiffBase<LeftAndRight_, Usage_>& lhs, const RotationDiffBase<LeftAndRight_, Usage_>& rhs) {
    return LeftAndRight_(typename LeftAndRight_::Implementation(lhs.derived().toImplementation() + rhs.derived().toImplementation()));
  }
  /*! \returns the subtraction of two angular velocities
   * \param lhs left-hand side
   * \param rhs right-hand side
   */
  inline static LeftAndRight_ subtract(const RotationDiffBase<LeftAndRight_, Usage_>& lhs, const RotationDiffBase<LeftAndRight_,Usage_>& rhs) {
    return LeftAndRight_(typename LeftAndRight_::Implementation(lhs.derived().toImplementation() - rhs.derived().toImplementation()));
  }
};




} // namespace internal

} // namespace rotations
} // namespace kindr




#endif /* KINDR_ROTATIONS_RDIFFBASE_HPP_ */
