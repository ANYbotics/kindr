/*
 * RotationBase.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef ROTATIONBASE_HPP_
#define ROTATIONBASE_HPP_

#include "rm/common/common.hpp"
#include "rm/quaternions/QuaternionBase.hpp"

namespace rm {

/*!
 *  \addtogroup Rotations
 *  @{
 */

//! Generic rotation interface
namespace rotations {
//! Internal stuff (only for developers)
namespace internal {

template<typename DEST, typename SOURCE>
class ConversionTraits {
public:
  // inline static DEST convert(const SOURCE & );
};

template<typename LEFT, typename RIGHT>
class MultiplicationTraits {
 public:
//  inline static LEFT mult(const LEFT & l, const RIGHT & r);
};


template<typename ROTATION>
class get_matrix3X {
  //  typedef int IndexType; // find type of Cols (e.g. Eigen::Dynamic)
  //  template <IndexType Cols> Matrix3X {
  //    typedef MATRIX type;
  //  }
};

template<typename ROTATION>
class RotationTraits {
  // inline static typename internal::get_vector3<DERIVED>::type rotate(const ROTATION & r, const typename internal::get_vector3<DERIVED>::type & );
};

template<typename ROTATION> // only works with the same rotation representation
class ComparisonTraits {
 public:
  inline static bool isEqual(const ROTATION & a, const ROTATION & b) {
    return a.toImplementation() == b.toImplementation();
  }

//  inline static bool areNearlyEqual(const eigen_implementation::RotationQuaternion<PrimType, Usage> & a, const eigen_implementation::RotationQuaternion<PrimType, Usage> & b, PrimType tol)
};

} // namespace internal

//! Representation of a generic rotation
/*!
 * @ingroup rotations
 */
template<typename DERIVED, enum RotationUsage Usage_>
class RotationBase {
 public:
  static constexpr enum RotationUsage Usage = Usage_;

  /*! Inverts the rotation
   *
   * @return inverse of the rotation
   */
  RotationBase inverse();

  RotationBase() = default;
  RotationBase(const DERIVED &) = delete; // on purpose!!

  operator DERIVED & () {
    return static_cast<DERIVED &>(*this);
  }
  operator const DERIVED & () const {
    return static_cast<const DERIVED &>(*this);
  }

  const DERIVED & derived() const {
    return static_cast<const DERIVED &>(*this);
  }

  template <typename internal::get_matrix3X<DERIVED>::IndexType Cols>
  typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> rotate(typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> & m) const {
    return internal::RotationTraits<DERIVED>::rotate(this->derived(), m);
  }

  template <typename internal::get_matrix3X<DERIVED>::IndexType Cols>
  typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> inverseRotate(typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> & m) const {
    return internal::RotationTraits<DERIVED>::rotate(this->derived().inverse(), m); // todo: may be optimized
  }

  template<typename OTHER_DERIVED>
  DERIVED operator *(const RotationBase<OTHER_DERIVED,Usage> & other) const {
    return internal::MultiplicationTraits<RotationBase<DERIVED,Usage>,RotationBase<OTHER_DERIVED,Usage>>::mult(this->derived(), other.derived()); // todo: 1. ok? 2. may be optimized
  }

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
};


//template<typename DERIVED, typename OTHER_DERIVED>
//bool isNear(const Rotation<DERIVED> & a, const Rotation<OTHER_DERIVED, Usage> & b, typename DERIVED::Scalar tol) { // todo: may be optimized
//  return internal::ComparisonTraits<DERIVED>::isNear(typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(a.derived()).getUnique(),
//                                                     typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(b.derived()).getUnique(),
//                                                     tol);
//}



template<typename Implementation, enum RotationUsage Usage>
class AngleAxisBase : public RotationBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  AngleAxisBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};

template<typename Implementation, enum RotationUsage Usage>
class RotationQuaternionBase : public RotationBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  RotationQuaternionBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};

template<typename Implementation, enum RotationUsage Usage>
class RotationMatrixBase : public RotationBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  RotationMatrixBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};

template<typename Implementation, enum RotationUsage Usage>
class EulerAnglesBase : public RotationBase<Implementation, Usage> {

};

template<typename Implementation, enum RotationUsage Usage>
class EulerAnglesXyzBase : public EulerAnglesBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  EulerAnglesXyzBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};

template<typename Implementation, enum RotationUsage Usage>
class EulerAnglesZyxBase : public EulerAnglesBase<Implementation, Usage> {

  template<typename OTHER_DERIVED>
  EulerAnglesZyxBase & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    return *this;
  }
};


namespace internal {

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
/*! @} End of Doxygen Groups*/
} // namespace rm

#endif /* ROTATIONBASE_HPP_ */
