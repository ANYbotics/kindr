/*
 * RotationBase.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef ROTATIONBASE_HPP_
#define ROTATIONBASE_HPP_

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
  // inline static DEST convert(const SOURCE & );
};

template<typename LEFT, typename RIGHT>
class MultiplicationTraits {
 public:
  inline static LEFT mult(const LEFT & l, const RIGHT & r) { // todo: improve
	return LEFT(typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar>(
		       (typename eigen_implementation::RotationQuaternion<typename LEFT::Scalar>(l)).toImplementation() *
		       (typename eigen_implementation::RotationQuaternion<typename RIGHT::Scalar>(r)).toImplementation()
		       ));
  }
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
  inline static bool areEqual(const ROTATION & a, const ROTATION & b) {
    return a.toImplementation() == b.toImplementation();
  }

//  inline static bool areNearlyEqual(const eigen_implementation::RotationQuaternion<PrimType> & a, const eigen_implementation::RotationQuaternion<PrimType> & b, PrimType tol)
};

} // namespace internal


//! Representation of a generic rotation
/*!
 * @ingroup rotations
 */
template<typename DERIVED>
class Rotation {
 public:

  /*! Inverts the rotation
   *
   * @return inverse of the rotation
   */
  Rotation inverse();

  Rotation() = default;
  Rotation(const DERIVED &) = delete; // on purpose!!

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
  DERIVED operator *(const Rotation<OTHER_DERIVED> & other) const {
    return internal::MultiplicationTraits<DERIVED,OTHER_DERIVED>::mult(this->derived(), other.derived()); // todo: 1. ok? 2. may be optimized
  }

  template<typename OTHER_DERIVED>
  bool operator ==(const Rotation<OTHER_DERIVED> & other) const { // todo: may be optimized
    return internal::ComparisonTraits<DERIVED>::isEqual(this->derived().getUnique(), DERIVED(other).getUnique()); // the type conversion must already take place here to ensure the specialised isequal function is called more often
  }

//  template<typename OTHER_DERIVED>
//  bool isNear(const Rotation<OTHER_DERIVED> & other, typename DERIVED::Scalar tol) { // todo: may be optimized
//    return internal::ComparisonTraits<DERIVED>::isNear(typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(this->derived()).getUnique(),
//                                                       typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(other.derived()).getUnique(),
//                                                       tol);
//  }
};


//template<typename DERIVED, typename OTHER_DERIVED>
//bool isNear(const Rotation<DERIVED> & a, const Rotation<OTHER_DERIVED> & b, typename DERIVED::Scalar tol) { // todo: may be optimized
//  return internal::ComparisonTraits<DERIVED>::isNear(typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(a.derived()).getUnique(),
//                                                     typename eigen_implementation::RotationQuaternion<typename DERIVED::Scalar>(b.derived()).getUnique(),
//                                                     tol);
//}



template<typename Implementation>
class AngleAxisBase : public Rotation<Implementation> {

  template<typename OTHER_DERIVED>
  AngleAxisBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }
};

template<typename Implementation>
class RotationQuaternionBase : public Rotation<Implementation> {

  template<typename OTHER_DERIVED>
  RotationQuaternionBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }

};

template<typename Implementation>
class RotationMatrixBase : public Rotation<Implementation> {

  template<typename OTHER_DERIVED>
  RotationMatrixBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }
};

template<typename Implementation>
class EulerAnglesBase : public Rotation<Implementation> {

};

template<typename Implementation>
class EulerAnglesXYZBase : public EulerAnglesBase<Implementation> {

  template<typename OTHER_DERIVED>
  EulerAnglesXYZBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }
};

template<typename Implementation>
class EulerAnglesZYXBase : public EulerAnglesBase<Implementation> {

  template<typename OTHER_DERIVED>
  EulerAnglesZYXBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }
};


} // namespace rotations
/*! @} End of Doxygen Groups*/
} // namespace rm

#endif /* ROTATIONBASE_HPP_ */
