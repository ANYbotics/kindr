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

#ifndef ROTATIONBASE_HPP_
#define ROTATIONBASE_HPP_

#include "rm/common/common.hpp"
#include "rm/quaternions/QuaternionBase.hpp"

namespace rm {
//! Generic rotation interface
/*! \ingroup rotations
 */
namespace rotations {
//! Internal stuff (only for developers)
namespace internal {

/*! \brief Usage conversion traits for converting active and passive rotations into each other
 *  \class UsageConversionTraits
 *  (only for advanced users)
 */
template<typename DERIVED, enum RotationUsage Usage_>
class UsageConversionTraits {
 public:
//  inline static typename get_other_usage<DERIVED>::OtherUsage getActive(const RotationBase<DERIVED,RotationUsage::PASSIVE> & in);
//  inline static typename get_other_usage<DERIVED>::OtherUsage getPassive(const RotationBase<DERIVED,RotationUsage::ACTIVE> & in);
};

/*! \brief This class determines the alternative usage type for each rotation
 *  \class get_matrix3X
 *  (only for advanced users)
 */
template<typename ROTATION>
class get_other_usage {
 public:
//  typedef eigen_implementation::AngleAxis<PrimType, RotationUsage::PASSIVE> OtherUsage;
};

/*! \brief Conversion traits for converting rotations into each other
 *  \class ConversionTraits
 *  (only for advanced users)
 */
template<typename DEST, typename SOURCE>
class ConversionTraits {
 public:
  // inline static DEST convert(const SOURCE & );
};

/*! \brief Comparison traits for comparing different rotations
 *  \class ComparisonTraits
 *  (only for advanced users)
 */
template<typename ROTATION> // only works with the same rotation representation
class ComparisonTraits {
 public:
  inline static bool isEqual(const ROTATION & a, const ROTATION & b) {
    return a.toImplementation() == b.toImplementation();
  }

//  inline static bool areNearlyEqual(const eigen_implementation::RotationQuaternion<PrimType, Usage> & a, const eigen_implementation::RotationQuaternion<PrimType, Usage> & b, PrimType tol)
};

/*! \brief Multiplication traits for concenating rotations
 *  \class MultiplicationTraits
 *  (only for advanced users)
 */
template<typename LEFT, typename RIGHT>
class MultiplicationTraits {
 public:
//  inline static LEFT mult(const LEFT & l, const RIGHT & r);
};

/*! \brief Rotation traits for rotating vectors and matrices
 *  \class RotationTraits
 *  (only for advanced users)
 */
template<typename ROTATION>
class RotationTraits {
 public:
// inline static typename internal::get_vector3<DERIVED>::type rotate(const ROTATION & r, const typename internal::get_vector3<DERIVED>::type & );
};

/*! \brief This class determines the correct matrix type for each rotation which is used for matrix rotations
 *  \class get_matrix3X
 *  (only for advanced users)
 */
template<typename ROTATION>
class get_matrix3X {
 public:
//  typedef int IndexType; // find type of Cols (e.g. Eigen::Dynamic)
//  template <IndexType Cols> Matrix3X {
//    typedef MATRIX type;
//  }
};

} // namespace internal



/*! \brief Representation of a generic rotation
 *  \ingroup rotations
 *  \class RotationBase
 *  This class defines the generic interface for a rotation.
 *  \tparam DERIVED the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename DERIVED, enum RotationUsage Usage_>
class RotationBase {
 public:
  /*! \brief Rotation usage.
   *  The rotation usage is either active (rotation of an object) or passive (transformation of its coordinates)
   */
  static constexpr enum RotationUsage Usage = Usage_;

  /*! \brief Standard constructor.
   *  Creates an empty generic rotation object
   */
  RotationBase() = default;

  /*! \brief Constructor from derived rotation.
   *  This constructor has been deleted because the abstract class does not contain any data.
   */
  RotationBase(const DERIVED &) = delete; // on purpose!!

  /*! \brief Returns the inverse of the rotation
   *  \returns inverse of the rotation
   */
  RotationBase inverted() const;

  /*! \brief Inverts the rotation.
   *
   */
  RotationBase & invert();

  /*! \brief Gets the derived rotation.
   *  (only for advanced users)
   *  \returns the derived rotation
   */
  operator DERIVED & () {
    return static_cast<DERIVED &>(*this);
  }

  /*! \brief Gets the derived rotation.
   *  (only for advanced users)
   *  \returns the derived rotation
   */
  operator const DERIVED & () const {
    return static_cast<const DERIVED &>(*this);
  }

  /*! \brief Gets the derived rotation.
   *  (only for advanced users)
   *  \returns the derived rotation
   */
  const DERIVED & derived() const {
    return static_cast<const DERIVED &>(*this);
  }

  /*! \brief Gets passive from active rotation.
   *  \returns the passive rotation
   */
  inline typename internal::get_other_usage<DERIVED>::OtherUsage getPassive() const {
    return internal::UsageConversionTraits<DERIVED,Usage>::getPassive(*this);
  }

  /*! \brief Gets active from passive rotation.
   *  \returns the active rotation
   */
  inline typename internal::get_other_usage<DERIVED>::OtherUsage getActive() const {
    return internal::UsageConversionTraits<DERIVED,Usage>::getActive(*this);
  }

  /*! \brief Concenates two rotations.
   *  \returns the concenations of two rotations
   */
  template<typename OTHER_DERIVED>
  DERIVED operator *(const RotationBase<OTHER_DERIVED,Usage> & other) const {
    return internal::MultiplicationTraits<RotationBase<DERIVED,Usage>,RotationBase<OTHER_DERIVED,Usage>>::mult(this->derived(), other.derived()); // todo: 1. ok? 2. may be optimized
  }

  /*! \brief Compares two rotations.
   *  \returns true if the rotations are exactly equal
   */
  template<typename OTHER_DERIVED>
  bool operator ==(const RotationBase<OTHER_DERIVED,Usage> & other) const { // todo: may be optimized
    return internal::ComparisonTraits<DERIVED>::isEqual(this->derived().getUnique(), DERIVED(other).getUnique()); // the type conversion must already take place here to ensure the specialised isequal function is called more often
  }

//  template<typename OTHER_DERIVED>
//  bool isNear(const Rotation<OTHER_DERIVED, Usage> & other, typename DERIVED::Scalar tol) { // todo: may be optimized
//    return internal::ComparisonTraits<DERIVED>::isNear(typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(this->derived()).getUnique(),
//                                                       typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(other.derived()).getUnique(),
//                                                       tol);
//  }

  /*! \brief Rotates a vector or matrix.
   *  \returns the rotated vector or matrix
   */
  template <typename internal::get_matrix3X<DERIVED>::IndexType Cols>
  typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> rotate(typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> & m) const {
    return internal::RotationTraits<DERIVED>::rotate(this->derived(), m);
  }

  /*! \brief Rotates a vector or matrix in reverse.
   *  \returns the reverse rotated vector or matrix
   */
  template <typename internal::get_matrix3X<DERIVED>::IndexType Cols>
  typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> inverseRotate(typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> & m) const {
    return internal::RotationTraits<DERIVED>::rotate(this->derived().inverse(), m); // todo: may be optimized
  }
};


//template<typename DERIVED, typename OTHER_DERIVED>
//bool isNear(const Rotation<DERIVED> & a, const Rotation<OTHER_DERIVED, Usage> & b, typename DERIVED::Scalar tol) { // todo: may be optimized
//  return internal::ComparisonTraits<DERIVED>::isNear(typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(a.derived()).getUnique(),
//                                                     typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(b.derived()).getUnique(),
//                                                     tol);
//}


/*! \brief Representation of a generic angle axis rotation
 *  \ingroup rotations
 *  \class AngleAxisBase
 *  This class defines the generic interface for an angle axis rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage>
class AngleAxisBase : public RotationBase<Implementation, Usage> {

  template<typename OTHER_DERIVED> // todo: necessary?
  AngleAxisBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};

/*! \brief Representation of a generic quaternion rotation
 *  \ingroup rotations
 *  \class RotationQuaternionBase
 *  This class defines the generic interface for a quaternion rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage>
class RotationQuaternionBase : public RotationBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  RotationQuaternionBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};

/*! \brief Representation of a generic matrix rotation
 *  \ingroup rotations
 *  \class RotationMatrixBase
 *  This class defines the generic interface for a matrix rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage>
class RotationMatrixBase : public RotationBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  RotationMatrixBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};

/*! \brief Representation of a generic euler angles rotation
 *  \ingroup rotations
 *  \class EulerAnglesBase
 *  This class defines the generic interface for a euler angles rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage>
class EulerAnglesBase : public RotationBase<Implementation, Usage> {

};

/*! \brief Representation of a generic euler angles xyz rotation
 *  \ingroup rotations
 *  \class EulerAnglesXyzBase
 *  This class defines the generic interface for a euler angles (X,Y',Z'' / roll,pitch,yaw) rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage>
class EulerAnglesXyzBase : public EulerAnglesBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  EulerAnglesXyzBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};

/*! \brief Representation of a generic euler angles zyx rotation
 *  \ingroup rotations
 *  \class EulerAnglesZyxBase
 *  This class defines the generic interface for a euler angles (Z,Y',X'' / yaw,pitch,roll) rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage>
class EulerAnglesZyxBase : public EulerAnglesBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  EulerAnglesZyxBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};


namespace internal {


template<typename DERIVED>
class UsageConversionTraits<DERIVED,RotationUsage::PASSIVE> {
 public:
  inline static typename get_other_usage<DERIVED>::OtherUsage getActive(const RotationBase<DERIVED,RotationUsage::PASSIVE> & in) {
    return typename get_other_usage<DERIVED>::OtherUsage(in.derived().inverted());
  }

  // getPassive() does not exist (on purpose)
};

template<typename DERIVED>
class UsageConversionTraits<DERIVED,RotationUsage::ACTIVE> {
 public:
  inline static typename get_other_usage<DERIVED>::OtherUsage getPassive(const RotationBase<DERIVED,RotationUsage::ACTIVE> & in) {
    return typename get_other_usage<DERIVED>::OtherUsage(in.derived().inverted());
  }

  // getActive() does not exist (on purpose)
};

//template<typename LEFT, typename RIGHT, enum RotationUsage Usage>
//class MultiplicationTraits<RotationBase<LEFT, Usage>, RotationBase<RIGHT, Usage>> {
//// public:
////  inline static LEFT mult(const RotationBase<LEFT, Usage> & l, const RotationBase<RIGHT, Usage> & r) {
////    return LEFT(typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar,  Usage>(
////               (typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar,  Usage>(l.derived())).toImplementation() *
////               (typename eigen_implementation::RotationQuaternion<typename RIGHT::Scalar, Usage>(r.derived())).toImplementation()
////               ));
////  }
//};
//
//template<typename LEFT, typename RIGHT>
//class MultiplicationTraits<RotationBase<LEFT, RotationUsage::ACTIVE>, RotationBase<RIGHT, RotationUsage::ACTIVE>> {
// public:
//  inline static LEFT mult(const RotationBase<LEFT, RotationUsage::ACTIVE> & l, const RotationBase<RIGHT, RotationUsage::ACTIVE> & r) {
//    return LEFT(typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar,  RotationUsage::ACTIVE>(
//               (typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar,  RotationUsage::ACTIVE>(l.derived())).toImplementation() *
//               (typename eigen_implementation::RotationQuaternion<typename RIGHT::Scalar, RotationUsage::ACTIVE>(r.derived())).toImplementation()
//               ));
//  }
//};
//
//template<typename LEFT, typename RIGHT>
//class MultiplicationTraits<RotationBase<LEFT, RotationUsage::PASSIVE>, RotationBase<RIGHT, RotationUsage::PASSIVE>> {
// public:
//  inline static LEFT mult(const RotationBase<LEFT, RotationUsage::PASSIVE> & l, const RotationBase<RIGHT, RotationUsage::PASSIVE> & r) {
//    return LEFT(typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar,  RotationUsage::PASSIVE>(
//               (typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar,  RotationUsage::PASSIVE>(l.derived())).toImplementation() *
//               (typename eigen_implementation::RotationQuaternion<typename RIGHT::Scalar, RotationUsage::PASSIVE>(r.derived())).toImplementation()
//               ));
//  }
//};
//
//template<typename LEFT_AND_RIGHT, enum RotationUsage Usage>
//class MultiplicationTraits<RotationBase<LEFT_AND_RIGHT, Usage>, RotationBase<LEFT_AND_RIGHT, Usage>> {
//// public:
////  inline static LEFT_AND_RIGHT mult(const RotationBase<LEFT_AND_RIGHT, Usage> & l, const RotationBase<LEFT_AND_RIGHT, Usage> & r) {
////    return LEFT_AND_RIGHT(typename LEFT_AND_RIGHT::Implementation(l.derived().toImplementation() * r.derived().toImplementation()));
////  }
//};
//
//template<typename LEFT_AND_RIGHT>
//class MultiplicationTraits<RotationBase<LEFT_AND_RIGHT, RotationUsage::ACTIVE>, RotationBase<LEFT_AND_RIGHT, RotationUsage::ACTIVE>> {
// public:
//  inline static LEFT_AND_RIGHT mult(const RotationBase<LEFT_AND_RIGHT, RotationUsage::ACTIVE> & l, const RotationBase<LEFT_AND_RIGHT, RotationUsage::ACTIVE> & r) {
//    return LEFT_AND_RIGHT(typename LEFT_AND_RIGHT::Implementation(l.derived().toImplementation() * r.derived().toImplementation()));
//  }
//};
//
//template<typename LEFT_AND_RIGHT>
//class MultiplicationTraits<RotationBase<LEFT_AND_RIGHT, RotationUsage::PASSIVE>, RotationBase<LEFT_AND_RIGHT, RotationUsage::PASSIVE>> {
// public:
//  inline static LEFT_AND_RIGHT mult(const RotationBase<LEFT_AND_RIGHT, RotationUsage::PASSIVE> & l, const RotationBase<LEFT_AND_RIGHT, RotationUsage::PASSIVE> & r) {
//    return LEFT_AND_RIGHT(typename LEFT_AND_RIGHT::Implementation(l.derived().toImplementation() * r.derived().toImplementation()));
//  }
//};

template<typename LEFT, typename RIGHT, enum RotationUsage Usage>
class MultiplicationTraits<RotationBase<LEFT, Usage>, RotationBase<RIGHT, Usage>> {
 public:
  inline static LEFT mult(const RotationBase<LEFT, Usage> & l, const RotationBase<RIGHT, Usage> & r) {
    return LEFT(typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar,  Usage>(
               (typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar,  Usage>(l.derived())).toImplementation() *
               (typename eigen_implementation::RotationQuaternion<typename RIGHT::Scalar, Usage>(r.derived())).toImplementation()
               ));
  }
};

template<typename LEFT_AND_RIGHT, enum RotationUsage Usage>
class MultiplicationTraits<RotationBase<LEFT_AND_RIGHT, Usage>, RotationBase<LEFT_AND_RIGHT, Usage>> {
 public:
  inline static LEFT_AND_RIGHT mult(const RotationBase<LEFT_AND_RIGHT, Usage> & l, const RotationBase<LEFT_AND_RIGHT, Usage> & r) {
    return LEFT_AND_RIGHT(typename LEFT_AND_RIGHT::Implementation(l.derived().toImplementation() * r.derived().toImplementation()));
  }
};

} // namespace internal
} // namespace rotations
} // namespace rm

#endif /* ROTATIONBASE_HPP_ */
