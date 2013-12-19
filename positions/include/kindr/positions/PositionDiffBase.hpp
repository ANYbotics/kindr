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

#ifndef KINDR_POSITIONS_PDIFFBASE_HPP_
#define KINDR_POSITIONS_PDIFFBASE_HPP_


#include "kindr/common/common.hpp"

namespace kindr {
namespace positions {
//! Internal stuff (only for developers)
namespace internal {

/*! \brief Addition traits for linear velocities
 *  \class PDiffAdditionTraits
 *  (only for advanced users)
 */
template<typename LeftAndRight_>
class PDiffAdditionTraits {
 public:
//  inline static LeftAndRight_ add(const LeftAndRight_& lhs, const LeftAndRight_& rhs);
//  inline static LeftAndRight_ subtract(const LeftAndRight_& lhs, const LeftAndRight_& rhs);
};

/*! \class get_scalar
 *  \brief Gets the primitive of the position.
 */
template<typename Position_>
class get_scalar {
 public:
//  typedef PrimType Scalar;
};

} // namespace internal

/*! \class PositionDiffBase
 * \brief Interface for a linear velocity.
 *
 * This class defines the generic interface for a linear velocity.
 * More precisely an interface to store and access the linear velocities of a point is provided.
 * \tparam Derived_ the derived class that should implement the linear velocity.
 * \ingroup positions
 *
 */
template<typename Derived_>
class PositionDiffBase {
 public:
  /*! \brief Default constructor.
   *
    *  Creates a linear velocity with zero coefficients.
    */
  PositionDiffBase() = default;

  /*! \brief Constructor from derived linear velocity.
   *
   *  This constructor has been deleted because the abstract class does not contain any data.
   */
  PositionDiffBase(const Derived_&) = delete; // on purpose!!

  /*! \brief Gets the derived linear velocity.
   *  (only for advanced users)
   *  \returns the derived linear velocity
   */
  operator Derived_& () {
    return static_cast<Derived_&>(*this);
  }

  /*! \brief Gets the derived linear velocity.
   *  (only for advanced users)
   *  \returns the derived linear velocity
   */
  operator const Derived_& () const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief Gets the derived linear velocity.
   *  (only for advanced users)
   *  \returns the derived linear velocity
   */
  const Derived_& derived() const {
    return static_cast<const Derived_&>(*this);
  }


  /*! \brief Sets the linear velocity to zero.
   *  \returns reference
   */
  Derived_& setZero();

  /*! \brief Addition of two linear velocities.
   *  \returns sum of the two linear velocities.
   */
  template<typename OtherDerived_>
  Derived_ operator +(const PositionDiffBase<OtherDerived_>& other) const {
    return internal::PDiffAdditionTraits<PositionDiffBase<Derived_>>::add(this->derived(), other.derived()); // todo: 1. ok? 2. may be optimized
  }

  /*! \brief Subtraction of two time derivatives.
   *  \returns result of the subtraction of the two two time derivatives.
   */
  template<typename OtherDerived_>
  Derived_ operator -(const PositionDiffBase<OtherDerived_>& other) const {
    return internal::PDiffAdditionTraits<PositionDiffBase<Derived_>>::subtract(this->derived(), other.derived()); // todo: 1. ok? 2. may be optimized
  }

  /*! \brief Addition and assignment.
   *  \returns sum of the two linear velocities.
   */
  template<typename OtherDerived_>
  Derived_& operator +=(const PositionDiffBase<OtherDerived_>& other);

  /*! \brief Subtraction and assignment.
   *  \returns result of the subtraction of the two linear velocities.
   */
  template<typename OtherDerived_>
  Derived_& operator -=(const PositionDiffBase<OtherDerived_>& other);

};

/*! \class LinearVelocity3Base
 * \brief Interface for a linear velocity in 3D-space.
 *
 * This class defines the generic interface for a linear velocity in 3D-space.
 * More precisely an interface to store and access the three linear velocities of a point is provided.

 * \tparam Derived_ the derived class that should implement the linear velocity.
 *
 *  \ingroup positions
 */
template<typename Derived_>
class LinearVelocity3Base : public PositionDiffBase<Derived_> {
 public:
  /*! \brief The primitive type of a linear velocity.
   */
  typedef typename internal::get_scalar<Derived_>::Scalar Scalar;

  /*! \returns the x-coordinate of the 3D linear velocity
   */
  inline const Scalar& x() const;

  /*! \returns the x-coordinate of the 3D linear velocity
   */
  inline Scalar& x();

  /*! \returns the y-coordinate of the 3D linear velocity
   */
  inline const Scalar& y() const;

  /*! \returns the y-coordinate of the 3D linear velocity
   */
  inline Scalar& y();

  /*! \returns the z-coordinate of the 3D-position
   */
  inline Scalar& z() const;

  /*! \returns the z-coordinate of the 3D linear velocity
   */
  inline Scalar& z();

};


namespace internal {

template<typename LeftAndRight_>
class PDiffAdditionTraits<PositionDiffBase<LeftAndRight_>> {
 public:
  /*! \returns the sum of two linear velocities
   * \param lhs left-hand side
   * \param rhs right-hand side
   */
  inline static LeftAndRight_ add(const PositionDiffBase<LeftAndRight_>& lhs, const PositionDiffBase<LeftAndRight_>& rhs) {
    return LeftAndRight_(typename LeftAndRight_::Implementation(lhs.derived().toImplementation() + rhs.derived().toImplementation()));
  }
  /*! \returns the subtraction of two linear velocities
   * \param lhs left-hand side
   * \param rhs right-hand side
   */
  inline static LeftAndRight_ subtract(const PositionDiffBase<LeftAndRight_>& lhs, const PositionDiffBase<LeftAndRight_>& rhs) {
    return LeftAndRight_(typename LeftAndRight_::Implementation(lhs.derived().toImplementation() - rhs.derived().toImplementation()));
  }
};

} // namespace internal

} // namespace positions
} // namespace kindr




#endif /* KINDR_POSITIONS_PDIFFBASE_HPP_ */
