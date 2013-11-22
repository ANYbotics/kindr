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
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#ifndef QUATERNIONBASE_HPP_
#define QUATERNIONBASE_HPP_

#include "rm/common/common.hpp"

namespace rm {
//! Generic quaternion interface
/*! \ingroup quaternions
 */
namespace quaternions {
//! Internal stuff (only for developers)
namespace internal {

//! Conversion trait to implement conversions between different types
template<typename DEST, typename SOURCE>
class ConversionTraits {
  // DEST convert(const SOURCE & );
};

/*! \brief Multiplication trait to implement quaternion multiplication
 * \class MultiplicationTraits
 */
template<typename LEFT, typename RIGHT>
class MultiplicationTraits {
 public:
  inline static typename LEFT::Implementation mult(const LEFT &l, const RIGHT &r){
    return typename LEFT::Implementation(l.toImplementation() * r.toImplementation());
  }
};

//! Comparision trait to implement to compare two quaterionsn
template<typename QUATERNION>
class ComparisonTraits {
 public:
  inline static bool isEqual(const QUATERNION & a, const QUATERNION & b){
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
 * \tparam DERIVED the derived class that should implement the quaternion
 * \see rm::rotations::RotationQuaternionBase for quaternions that represent a rotation
 */
template<typename DERIVED>
class QuaternionBase {
 public:
  /*!\brief inverts the quaternion
   * \returns the inverted quaternion
   */
  DERIVED & invert();

  /*! \brief gets the inverse of the quaternion
   * \returns the inverse of the quaternion
    */
  DERIVED inverted() const;

  /*! \brief conjugates the quaternion
   *  \returns the conjugate of the quaternion
   */
  DERIVED & conjugate();

  /*!\brief gets the conjugate of the quaternion
   * \returns the conjugate of the quaternion
   */
  DERIVED conjugated() const;

  /*! \brief gets the derived quaternion
   *  (only for advanced users)
   * \returns the derived quaternion
   */
  operator DERIVED & () {
    return static_cast<DERIVED &>(*this);
  }

  /*! \brief gets the derived quaternion
   *  (only for advanced users)
   * \returns the derived quaternion
   */
  operator const DERIVED & () const {
    return static_cast<const DERIVED &>(*this);
  }
  /*! \brief gets the derived quaternion
   *  (only for advanced users)
   * \returns the derived quaternion
   */
  DERIVED & derived() {
    return static_cast<DERIVED &>(*this);
  }
  /*! \brief gets the derived quaternion
   *  (only for advanced users)
   * \returns the derived quaternion
   */
  const DERIVED & derived() const {
    return static_cast<const DERIVED &>(*this);
  }

  /*! \brief multiplies the quaternion with another quaternion
   * \return the product of two quaternions
   * \param other   other  other quaternion
   */
  template<typename OTHER_DERIVED>
  DERIVED operator *(const QuaternionBase<OTHER_DERIVED> & other) const {
    return DERIVED(internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(this->derived(), other.derived()));
  }

//  template<typename OTHER_DERIVED>
//  DERIVED operator *(const QuaternionBase<OTHER_DERIVED> & other) const {
//    return internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(this->derived(), static_cast<DERIVED>(other));
//  }

  /*! \brief compares the quaternion with another quaternion
   * \param other   other quaternion
   * \returns true if the quaternions are equal
   */
  template<typename OTHER_DERIVED>
  bool operator ==(const QuaternionBase<OTHER_DERIVED> & other) const {
    return internal::ComparisonTraits<DERIVED>::isEqual(this->derived(), static_cast<DERIVED>(other)); // cast to Quaternion
  }

  /*! \brief prints the coefficients of the quaternion
   * \returns w x y z
   */
  friend std::ostream & operator << (std::ostream & out, const QuaternionBase<DERIVED> & quat) {
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
template<typename DERIVED>
class UnitQuaternionBase : public QuaternionBase<DERIVED> {
 public:
  typedef QuaternionBase<DERIVED> Base;

  /*! \brief gets the inverse of the unit quaternion using the conjugate
   * \returns the inverse of the unit quaternion
    */
  DERIVED inverted() const {
    return Base::derived().conjugated();
  }

  /*!\brief inverts the unit quaternion using the conjugate
   * \returns the inverted unit quaternion
   */
  DERIVED & invert() {
    return Base::derived().conjugate();
  }
  /*!\brief gets the conjugate of the quaternion
   * \returns the conjugate of the quaternion
   */
  DERIVED conjugated() const;

  /*! \brief conjugates the unit quaternion
   *  \returns the conjugate of the unit quaternion
   */
  DERIVED & conjugate();

  /*! \brief multiplies the unit quaternion with another unit quaternion
   * \return the product of two unit quaternions
   * \param other   other unit quaternion
   */
  template<typename OTHER_DERIVED>
  DERIVED operator *(const UnitQuaternionBase<OTHER_DERIVED> & other) const {
    return DERIVED(internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(this->derived(), other.derived()));
  }

  /*! \brief multiplies the unit quaternion with a quaternion
   * \returns the product of two quaternions
   * \param other   other  quaternion
   */
  template<typename OTHER_DERIVED>
  OTHER_DERIVED operator *(const QuaternionBase<OTHER_DERIVED> & other) const {
    return OTHER_DERIVED(internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(this->derived(), other.derived()));
  }
};




} // namespace quaternions
} // namespace rm

#endif /* QUATERNIONBASE_HPP_ */
