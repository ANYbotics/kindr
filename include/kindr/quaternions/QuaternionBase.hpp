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

namespace kindr {

//! Internal stuff (only for developers)
namespace quat_internal {

//! Conversion trait to implement conversions between different types
template<typename Dest_, typename Source_>
class ConversionTraits {
  // Dest_ convert(const Source_& );
};

/*! \brief Multiplication trait to implement quaternion multiplication
 * \class MultiplicationTraits
 */
template<typename Left_, typename Right_>
class MultiplicationTraits {
 public:
  inline static typename Left_::Implementation mult(const Left_& lhs, const Right_& rhs){
    return typename Left_::Implementation(lhs.toImplementation() * rhs.toImplementation());
  }
};

//! Comparison trait to implement to compare two quaternions
template<typename Quaternion_>
class ComparisonTraits {
 public:
  inline static bool isEqual(const Quaternion_& a, const Quaternion_& b){
    return (a.w() ==  b.w() &&
	          a.x() ==  b.x() &&
	          a.y() ==  b.y() &&
	          a.z() ==  b.z());
  }
};

} // namespace internal

/*! \ingroup quaternions
 * \class QuaternionBase
 * \brief Base class that defines the interface of a quaternion
 *  This class defines a generic interface for a quaternion.
 * \tparam Derived_ the derived class that should implement the quaternion
 * \see rm::rotations::RotationQuaternionBase for quaternions that represent a rotation
 */
template<typename Derived_>
class QuaternionBase {
 public:
  /*!\brief inverts the quaternion
   * \returns the inverted quaternion
   */
  Derived_& invert();

  /*! \brief gets the inverse of the quaternion
   * \returns the inverse of the quaternion
    */
  Derived_ inverted() const;

  /*! \brief conjugates the quaternion
   *  \returns the conjugate of the quaternion
   */
  Derived_& conjugate();

  /*!\brief gets the conjugate of the quaternion
   * \returns the conjugate of the quaternion
   */
  Derived_ conjugated() const;

  /*! \brief gets the derived quaternion
   *  (only for advanced users)
   * \returns the derived quaternion
   */
  operator Derived_& () {
    return static_cast<Derived_&>(*this);
  }

  /*! \brief gets the derived quaternion
   *  (only for advanced users)
   * \returns the derived quaternion
   */
  operator const Derived_& () const {
    return static_cast<const Derived_&>(*this);
  }
  /*! \brief gets the derived quaternion
   *  (only for advanced users)
   * \returns the derived quaternion
   */
  Derived_& derived() {
    return static_cast<Derived_&>(*this);
  }
  /*! \brief gets the derived quaternion
   *  (only for advanced users)
   * \returns the derived quaternion
   */
  const Derived_& derived() const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief multiplies the quaternion with another quaternion
   * \return the product of two quaternions
   * \param other   other  other quaternion
   */
  template<typename OtherDerived_>
  Derived_ operator *(const QuaternionBase<OtherDerived_>& other) const {
    return Derived_(quat_internal::MultiplicationTraits<Derived_, OtherDerived_>::mult(this->derived(), other.derived()));
  }

//  template<typename OtherDerived_>
//  Derived_ operator *(const QuaternionBase<OtherDerived_>& other) const {
//    return quat_internal::MultiplicationTraits<Derived_, OtherDerived_>::mult(this->derived(), static_cast<Derived_>(other));
//  }

  /*! \brief compares the quaternion with another quaternion for equality
   * \param other   other quaternion
   * \returns true if the quaternions are equal
   */
  template<typename OtherDerived_>
  bool operator ==(const QuaternionBase<OtherDerived_>& other) const {
    return quat_internal::ComparisonTraits<Derived_>::isEqual(this->derived(), static_cast<Derived_>(other)); // cast to Quaternion
  }

  /*! \brief compares the quaternion with another quaternion for inequality
   * \param other   other quaternion
   * \returns true if the quaternions are not equal
   */
  template<typename OtherDerived_>
  bool operator !=(const QuaternionBase<OtherDerived_>& other) const {
    return !(*this == other);
  }

  /*! \brief prints the coefficients of the quaternion
   * \returns w x y z
   */
  friend std::ostream& operator << (std::ostream& out, const QuaternionBase<Derived_>& quat) {
    out << quat.derived().w() << " " << quat.derived().x() << " " << quat.derived().y() << " " << quat.derived().z();
    return out;
  }
};



/*! \ingroup quaternions
 * \class UnitQuaternionBase
 * \brief Base class that defines the interface of a unit quaternion
 * This class defines a generic interface for a unit quaternion.
 * \see rm::rotations::RotationQuaternionBase for quaternions that represent a rotation
 */
template<typename Derived_>
class UnitQuaternionBase : public QuaternionBase<Derived_> {
 public:
  typedef QuaternionBase<Derived_> Base;

  /*! \brief gets the inverse of the unit quaternion using the conjugate
   * \returns the inverse of the unit quaternion
    */
  Derived_ inverted() const {
    return Base::derived().conjugated();
  }

  /*!\brief inverts the unit quaternion using the conjugate
   * \returns the inverted unit quaternion
   */
  Derived_& invert() {
    return Base::derived().conjugate();
  }
  /*!\brief gets the conjugate of the quaternion
   * \returns the conjugate of the quaternion
   */
  Derived_ conjugated() const;

  /*! \brief conjugates the unit quaternion
   *  \returns the conjugate of the unit quaternion
   */
  Derived_& conjugate();

  /*! \brief multiplies the unit quaternion with another unit quaternion
   * \return the product of two unit quaternions
   * \param other   other unit quaternion
   */
  template<typename OtherDerived_>
  Derived_ operator *(const UnitQuaternionBase<OtherDerived_>& other) const {
    return Derived_(quat_internal::MultiplicationTraits<Derived_, OtherDerived_>::mult(this->derived(), other.derived()));
  }

  /*! \brief multiplies the unit quaternion with a quaternion
   * \returns the product of two quaternions
   * \param other   other  quaternion
   */
  template<typename OtherDerived_>
  OtherDerived_ operator *(const QuaternionBase<OtherDerived_>& other) const {
    return OtherDerived_(quat_internal::MultiplicationTraits<Derived_, OtherDerived_>::mult(this->derived(), other.derived()));
  }
};


} // namespace rm
