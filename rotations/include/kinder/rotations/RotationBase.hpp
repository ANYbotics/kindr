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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRight_ HOLDERS AND CONTRIBUTORS "AS IS" AND
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

#ifndef KINDER_ROTATIONS_ROTATION_BASE_HPP_
#define KINDER_ROTATIONS_ROTATION_BASE_HPP_

#include "kinder/common/common.hpp"
#include "kinder/quaternions/QuaternionBase.hpp"



namespace kinder {
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
template<typename Derived_, enum RotationUsage Usage_>
class UsageConversionTraits {
 public:
//  inline static typename get_other_usage<Derived_>::OtherUsage getActive(const RotationBase<Derived_,RotationUsage::PASSIVE>& in);
//  inline static typename get_other_usage<Derived_>::OtherUsage getPassive(const RotationBase<Derived_,RotationUsage::ACTIVE>& in);
};

/*! \brief This class determines the alternative usage type for each rotation
 *  \class get_matrix3X
 *  (only for advanced users)
 */
template<typename Rotation_>
class get_other_usage {
 public:
//  typedef eigen_implementation::AngleAxis<PrimType, RotationUsage::PASSIVE> OtherUsage;
};

/*! \brief Conversion traits for converting rotations into each other
 *  \class ConversionTraits
 *  (only for advanced users)
 */
template<typename Dest_, typename Source_>
class ConversionTraits {
 public:
  // inline static Dest_ convert(const Source_& );
};

/*! \brief Comparison traits for comparing different rotations
 *  \class ComparisonTraits
 *  (only for advanced users)
 */
template<typename Rotation_> // only works with the same rotation representation
class ComparisonTraits {
 public:
  inline static bool isEqual(const Rotation_& a, const Rotation_& b) {
    return a.toImplementation() == b.toImplementation();
  }

//  inline static bool areNearlyEqual(const eigen_implementation::RotationQuaternion<PrimType, Usage_>& a, const eigen_implementation::RotationQuaternion<PrimType, Usage_>& b, PrimType tol)
};

/*! \brief Multiplication traits for concenating rotations
 *  \class MultiplicationTraits
 *  (only for advanced users)
 */
template<typename Left_, typename Right_>
class MultiplicationTraits {
 public:
//  inline static Left_ mult(const Left_& l, const Right_& r);
};

/*! \brief Rotation traits for rotating vectors and matrices
 *  \class RotationTraits
 *  (only for advanced users)
 */
template<typename Rotation_>
class RotationTraits {
 public:
// inline static typename internal::get_vector3<Derived_>::type rotate(const Rotation_& r, const typename internal::get_vector3<Derived_>::type& );
};

/*! \brief This class determines the correct matrix type for each rotation which is used for matrix rotations
 *  \class get_matrix3X
 *  (only for advanced users)
 */
template<typename Rotation_>
class get_matrix3X {
 public:
//  typedef int IndexType; // find type of Cols (e.g. Eigen::Dynamic)
//  template <IndexType Cols> Matrix3X {
//    typedef MATRIX type;
//  }
};

/*! \brief This class determines the correct matrix type for each rotation which is used for position rotations
 *  \class get_matrix3
 *  (only for advanced users)
 */
template<typename Position_>
class get_matrix3 {
 public:
//  static const Matrix3X& getMatrix3(const Position& position) {
//    return position.toImplementation();
//  }
};



} // namespace internal



/*! \brief Representation of a generic rotation
 *  \ingroup rotations
 *  \class RotationBase
 *  This class defines the generic interface for a rotation.
 *  \tparam Derived_ the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Derived_, enum RotationUsage Usage_>
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
  RotationBase(const Derived_&) = delete; // on purpose!!

  /*! \brief Returns the inverse of the rotation
   *  \returns inverse of the rotation
   */
  RotationBase inverted() const;

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  RotationBase& invert();

  /*! \brief Gets the derived rotation.
   *  (only for advanced users)
   *  \returns the derived rotation
   */
  operator Derived_& () {
    return static_cast<Derived_&>(*this);
  }

  /*! \brief Gets the derived rotation.
   *  (only for advanced users)
   *  \returns the derived rotation
   */
  operator const Derived_& () const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief Gets the derived rotation.
   *  (only for advanced users)
   *  \returns the derived rotation
   */
  const Derived_& derived() const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief Sets the rotation to the identity rotation.
   *  \returns reference
   */
  Derived_& setIdentity();

  /*! \brief Returns the rotation in a unique form
   *  This function is used to compare different rotations.
   *  \returns copy of the rotation which is unique
   */
  Derived_ getUnique() const;

  /*! \brief  Modifies the rotation such that it is in its unique form
   *  \returns reference
   */
  Derived_& setUnique();

  /*! \brief Gets passive from active rotation.
   *  \returns the passive rotation
   */
  inline typename internal::get_other_usage<Derived_>::OtherUsage getPassive() const {
    return internal::UsageConversionTraits<Derived_,Usage_>::getPassive(*this);
  }

  /*! \brief Gets active from passive rotation.
   *  \returns the active rotation
   */
  inline typename internal::get_other_usage<Derived_>::OtherUsage getActive() const {
    return internal::UsageConversionTraits<Derived_,Usage_>::getActive(*this);
  }

  /*! \brief Concenates two rotations.
   *  \returns the concenations of two rotations
   */
  template<typename OtherDerived_>
  Derived_ operator *(const RotationBase<OtherDerived_,Usage_>& other) const {
    return internal::MultiplicationTraits<RotationBase<Derived_,Usage_>,RotationBase<OtherDerived_,Usage_>>::mult(this->derived(), other.derived()); // todo: 1. ok? 2. may be optimized
  }

  /*! \brief Compares two rotations.
   *  \returns true if the rotations are exactly equal
   */
  template<typename OtherDerived_>
  bool operator ==(const RotationBase<OtherDerived_,Usage_>& other) const { // todo: may be optimized
    return internal::ComparisonTraits<Derived_>::isEqual(this->derived().getUnique(), Derived_(other).getUnique()); // the type conversion must already take place here to ensure the specialised isequal function is called more often
  }

//  template<typename OtherDerived_>
//  bool isNear(const Rotation<OtherDerived_, Usage_>& other, typename Derived_::Scalar tol) { // todo: may be optimized
//    return internal::ComparisonTraits<Derived_>::isNear(typename eigen_implementation::RotationQuaternion<typename Derived_::Scalar>(this->derived()).getUnique(),
//                                                       typename eigen_implementation::RotationQuaternion<typename Derived_::Scalar>(other.derived()).getUnique(),
//                                                       tol);
//  }

  /*! \brief Rotates a vector or matrix.
   *  \returns the rotated vector or matrix
   */
  template <typename internal::get_matrix3X<Derived_>::IndexType Cols>
  typename internal::get_matrix3X<Derived_>::template Matrix3X<Cols> rotate(const typename internal::get_matrix3X<Derived_>::template Matrix3X<Cols>& matrix) const {
    return internal::RotationTraits<Derived_>::rotate(this->derived(), matrix);
  }

  /*! \brief Rotates a vector or matrix in reverse.
   *  \returns the reverse rotated vector or matrix
   */
  template <typename internal::get_matrix3X<Derived_>::IndexType Cols>
  typename internal::get_matrix3X<Derived_>::template Matrix3X<Cols> inverseRotate(const typename internal::get_matrix3X<Derived_>::template Matrix3X<Cols>& matrix) const {
    return internal::RotationTraits<Derived_>::rotate(this->derived().inverted(), matrix); // todo: may be optimized
  }

  /*! \brief Rotates a position.
   *  \returns the rotated position
   */
  template <typename Position_>
  Position_ rotate(const Position_& position) const {
    return Position_(internal::RotationTraits<Derived_>::rotate(this->derived(), internal::get_matrix3<Position_>::getMatrix3(position)));
  }
  /*! \brief Rotates a position in reverse.
   *  \returns the reverse rotated position
   */
  template <typename Position_>
  Position_ inverseRotate(const Position_& position) const {
    return Position_(internal::RotationTraits<Derived_>::rotate(this->derived().inverted(), internal::get_matrix3<Position_>::getMatrix3(position)));
  }



};


//template<typename Derived_, typename OtherDerived_>
//bool isNear(const Rotation<Derived_>& a, const Rotation<OtherDerived_, Usage_>& b, typename Derived_::Scalar tol) { // todo: may be optimized
//  return internal::ComparisonTraits<Derived_>::isNear(typename eigen_implementation::RotationQuaternion<typename Derived_::Scalar>(a.derived()).getUnique(),
//                                                     typename eigen_implementation::RotationQuaternion<typename Derived_::Scalar>(b.derived()).getUnique(),
//                                                     tol);
//}


/*! \brief Representation of a generic angle axis rotation
 *  \ingroup rotations
 *  \class AngleAxisBase
 *  This class defines the generic interface for an angle axis rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage_>
class AngleAxisBase : public RotationBase<Implementation, Usage_> {

  template<typename OtherDerived_> // todo: necessary?
  AngleAxisBase& operator =(const RotationBase<OtherDerived_, Usage_>& other);
};

/*! \brief Representation of a generic quaternion rotation
 *  \ingroup rotations
 *  \class RotationQuaternionBase
 *  This class defines the generic interface for a quaternion rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage_>
class RotationQuaternionBase : public RotationBase<Implementation, Usage_> {

  template<typename OtherDerived_>
  RotationQuaternionBase& operator =(const RotationBase<OtherDerived_, Usage_>& other);
};

/*! \brief Representation of a generic matrix rotation
 *  \ingroup rotations
 *  \class RotationMatrixBase
 *  This class defines the generic interface for a matrix rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage_>
class RotationMatrixBase : public RotationBase<Implementation, Usage_> {

  template<typename OtherDerived_>
  RotationMatrixBase& operator =(const RotationBase<OtherDerived_, Usage_>& other);

};

/*! \brief Representation of a generic euler angles rotation
 *  \ingroup rotations
 *  \class EulerAnglesBase
 *  This class defines the generic interface for a euler angles rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage_>
class EulerAnglesBase : public RotationBase<Implementation, Usage_> {

};

/*! \brief Representation of a generic euler angles xyz rotation
 *  \ingroup rotations
 *  \class EulerAnglesXyzBase
 *  This class defines the generic interface for a euler angles (X,Y',Z'' / roll,pitch,yaw) rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage_>
class EulerAnglesXyzBase : public EulerAnglesBase<Implementation, Usage_> {

  template<typename OtherDerived_>
  EulerAnglesXyzBase& operator =(const RotationBase<OtherDerived_, Usage_>& other);

};

/*! \brief Representation of a generic euler angles zyx rotation
 *  \ingroup rotations
 *  \class EulerAnglesZyxBase
 *  This class defines the generic interface for a euler angles (Z,Y',X'' / yaw,pitch,roll) rotation.
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation, enum RotationUsage Usage_>
class EulerAnglesZyxBase : public EulerAnglesBase<Implementation, Usage_> {

  template<typename OtherDerived_>
  EulerAnglesZyxBase& operator =(const RotationBase<OtherDerived_, Usage_>& other);

};


namespace internal {


template<typename Derived_>
class UsageConversionTraits<Derived_,RotationUsage::PASSIVE> {
 public:
  inline static typename get_other_usage<Derived_>::OtherUsage getActive(const RotationBase<Derived_,RotationUsage::PASSIVE>& in) {
    return typename get_other_usage<Derived_>::OtherUsage(in.derived().inverted());
  }

  // getPassive() does not exist (on purpose)
};

template<typename Derived_>
class UsageConversionTraits<Derived_,RotationUsage::ACTIVE> {
 public:
  inline static typename get_other_usage<Derived_>::OtherUsage getPassive(const RotationBase<Derived_,RotationUsage::ACTIVE>& in) {
    return typename get_other_usage<Derived_>::OtherUsage(in.derived().inverted());
  }

  // getActive() does not exist (on purpose)
};

//template<typename Left_, typename Right_, enum RotationUsage Usage_>
//class MultiplicationTraits<RotationBase<Left_, Usage_>, RotationBase<Right_, Usage_>> {
//// public:
////  inline static Left_ mult(const RotationBase<Left_, Usage_>& l, const RotationBase<Right_, Usage_>& r) {
////    return Left_(typename eigen_implementation::RotationQuaternion<typename Left_::Scalar,  Usage_>(
////               (typename eigen_implementation::RotationQuaternion<typename Left_::Scalar,  Usage_>(l.derived())).toImplementation() *
////               (typename eigen_implementation::RotationQuaternion<typename Right_::Scalar, Usage_>(r.derived())).toImplementation()
////               ));
////  }
//};
//
//template<typename Left_, typename Right_>
//class MultiplicationTraits<RotationBase<Left_, RotationUsage::ACTIVE>, RotationBase<Right_, RotationUsage::ACTIVE>> {
// public:
//  inline static Left_ mult(const RotationBase<Left_, RotationUsage::ACTIVE>& l, const RotationBase<Right_, RotationUsage::ACTIVE>& r) {
//    return Left_(typename eigen_implementation::RotationQuaternion<typename Left_::Scalar,  RotationUsage::ACTIVE>(
//               (typename eigen_implementation::RotationQuaternion<typename Left_::Scalar,  RotationUsage::ACTIVE>(l.derived())).toImplementation() *
//               (typename eigen_implementation::RotationQuaternion<typename Right_::Scalar, RotationUsage::ACTIVE>(r.derived())).toImplementation()
//               ));
//  }
//};
//
//template<typename Left_, typename Right_>
//class MultiplicationTraits<RotationBase<Left_, RotationUsage::PASSIVE>, RotationBase<Right_, RotationUsage::PASSIVE>> {
// public:
//  inline static Left_ mult(const RotationBase<Left_, RotationUsage::PASSIVE>& l, const RotationBase<Right_, RotationUsage::PASSIVE>& r) {
//    return Left_(typename eigen_implementation::RotationQuaternion<typename Left_::Scalar,  RotationUsage::PASSIVE>(
//               (typename eigen_implementation::RotationQuaternion<typename Left_::Scalar,  RotationUsage::PASSIVE>(l.derived())).toImplementation() *
//               (typename eigen_implementation::RotationQuaternion<typename Right_::Scalar, RotationUsage::PASSIVE>(r.derived())).toImplementation()
//               ));
//  }
//};
//
//template<typename LeftAndRight_, enum RotationUsage Usage_>
//class MultiplicationTraits<RotationBase<LeftAndRight_, Usage_>, RotationBase<LeftAndRight_, Usage_>> {
//// public:
////  inline static LeftAndRight_ mult(const RotationBase<LeftAndRight_, Usage_>& l, const RotationBase<LeftAndRight_, Usage_>& r) {
////    return LeftAndRight_(typename LeftAndRight_::Implementation(l.derived().toImplementation() * r.derived().toImplementation()));
////  }
//};
//
//template<typename LeftAndRight_>
//class MultiplicationTraits<RotationBase<LeftAndRight_, RotationUsage::ACTIVE>, RotationBase<LeftAndRight_, RotationUsage::ACTIVE>> {
// public:
//  inline static LeftAndRight_ mult(const RotationBase<LeftAndRight_, RotationUsage::ACTIVE>& l, const RotationBase<LeftAndRight_, RotationUsage::ACTIVE>& r) {
//    return LeftAndRight_(typename LeftAndRight_::Implementation(l.derived().toImplementation() * r.derived().toImplementation()));
//  }
//};
//
//template<typename LeftAndRight_>
//class MultiplicationTraits<RotationBase<LeftAndRight_, RotationUsage::PASSIVE>, RotationBase<LeftAndRight_, RotationUsage::PASSIVE>> {
// public:
//  inline static LeftAndRight_ mult(const RotationBase<LeftAndRight_, RotationUsage::PASSIVE>& l, const RotationBase<LeftAndRight_, RotationUsage::PASSIVE>& r) {
//    return LeftAndRight_(typename LeftAndRight_::Implementation(l.derived().toImplementation() * r.derived().toImplementation()));
//  }
//};

template<typename Left_, typename Right_, enum RotationUsage Usage_>
class MultiplicationTraits<RotationBase<Left_, Usage_>, RotationBase<Right_, Usage_>> {
 public:
  inline static Left_ mult(const RotationBase<Left_, Usage_>& lhs, const RotationBase<Right_, Usage_>& rhs) {
    return Left_(typename eigen_implementation::RotationQuaternion<typename Left_::Scalar,  Usage_>(
               (typename eigen_implementation::RotationQuaternion<typename Left_::Scalar,  Usage_>(lhs.derived())).toImplementation() *
               (typename eigen_implementation::RotationQuaternion<typename Right_::Scalar, Usage_>(rhs.derived())).toImplementation()
               ));
  }
};

template<typename LeftAndRight_, enum RotationUsage Usage_>
class MultiplicationTraits<RotationBase<LeftAndRight_, Usage_>, RotationBase<LeftAndRight_, Usage_>> {
 public:
  inline static LeftAndRight_ mult(const RotationBase<LeftAndRight_, Usage_>& lhs, const RotationBase<LeftAndRight_, Usage_>& rhs) {
    return LeftAndRight_(typename LeftAndRight_::Implementation(lhs.derived().toImplementation() * rhs.derived().toImplementation()));
  }
};

} // namespace internal
} // namespace rotations
} // namespace rm

#endif /* KINDER_ROTATIONS_ROTATION_BASE_HPP_ */
