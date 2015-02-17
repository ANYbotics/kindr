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

#ifndef KINDR_ROTATIONS_ROTATION_BASE_HPP_
#define KINDR_ROTATIONS_ROTATION_BASE_HPP_

#include "kindr/common/common.hpp"
#include "kindr/quaternions/QuaternionBase.hpp"
#include "kindr/vectors/VectorBase.hpp"


namespace kindr {
//! Generic rotation interface
/*! \ingroup rotations
 */
namespace rotations {

/*! \brief This enum class determines the usage type for each rotation
 *  \class RotationUsage
 */
enum class RotationUsage {
  ACTIVE,
  PASSIVE
};

//! Internal stuff (only for developers)
namespace internal {

/*! \brief This class determines the alternative usage type for each rotation
 *  \class get_other_usage
 *  (only for advanced users)
 */
template<typename Rotation_>
class get_other_usage {
 public:
//  typedef eigen_impl::AngleAxis<PrimType, RotationUsage::PASSIVE> OtherUsage;
};

template<typename Rotation_>
class get_scalar {
 public:
//  typedef typename Rotation_::Scalar Scalar;
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
template<typename Left_, typename Right_>
class ComparisonTraits {
 public:
  inline static bool isEqual(const Left_& left, const Right_& right) {
    return left.toImplementation() == Left_(right).toImplementation();
  }
};

/*! \brief Multiplication traits for concatenating rotations
 *  \class MultiplicationTraits
 *  (only for advanced users)
 */
template<typename Left_, typename Right_>
class MultiplicationTraits {
 public:
//  inline static Left_ mult(const Left_& lhs, const Right_& rhs);
};

/*! \brief Rotation traits for rotating vectors and matrices
 *  \class RotationTraits
 *  (only for advanced users)
 */
template<typename Rotation_>
class RotationTraits {
 public:
// inline static typename internal::get_matrix3X<Derived_>::type rotate(const Rotation_& r, const typename internal::get_vector3<Derived_>::type& );
};

/*! \brief Traits for map operations
 *  \class MapTraits
 *  (only for advanced users)
 */
template<typename Rotation_>
class MapTraits {
 public:
//  Derived_& setExponentialMap(const typename internal::get_matrix3X<Derived_>::template Matrix3X<1>& vector);
//  typename internal::get_matrix3X<Derived_>::template Matrix3X<1> getLogarithmicMap();
};

/*! \brief Traits for box operations
 *  \class BoxOperationTraits
 *  (only for advanced users)
 */
template<typename Left_, typename Right_>
class BoxOperationTraits {
 public:
//  inline static typename internal::get_matrix3X<Left_>::template Matrix3X<1> box_minus(const RotationBase<Left_, Usage_>& lhs, const RotationBase<Right_, Usage_>& rhs);
//  inline static Left_ box_plus(const RotationBase<Left_, Usage_>& rotation, const typename internal::get_matrix3X<Left_>::template Matrix3X<1>& vector);
};

/*! \brief Sets the rotation from two vectors such that v2 = R(v1).
 *  \class RotationTraits
 *  (only for advanced users)
 */
template<typename Rotation_>
class SetFromVectorsTraits {
 public:
};

/*! \brief Fixes the rotation to get rid of numerical errors (e.g. normalize quaternion).
 *  \class RotationTraits
 *  (only for advanced users)
 */
template<typename Rotation_>
class FixingTraits {
 public:
  inline static void fix(Rotation_& rot) {
    // do nothing is the standard case
  }
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
  Derived_& derived() {
    return static_cast<Derived_&>(*this);
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

  /*! \brief Concatenates two rotations.
   *  \returns the concatenation of two rotations
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
    return internal::ComparisonTraits<Derived_,OtherDerived_>::isEqual(this->derived().getUnique(), other.derived().getUnique()); // the type conversion must already take place here to ensure the specialised isequal function is called more often
  }

  /*! \brief Gets the disparity angle between two rotations for comparison.
   *
   *  The disparity angle is defined as the angle of the angle-axis representation of the concatenation of
   *  the first rotation and the inverse of the second rotation. If the disparity angle is zero,
   *  the rotations are equal.
   *
   *  \returns disparity angle
   */
  template<typename OtherDerived_>
  typename internal::get_scalar<Derived_>::Scalar getDisparityAngle(const RotationBase<OtherDerived_,Usage_>& other) const {
    return internal::ComparisonTraits<RotationBase<Derived_, Usage_>, RotationBase<OtherDerived_, Usage_>>::get_disparity_angle(this->derived(), other.derived());
  }

  /*! \brief Compares two rotations with a tolerance.
   *  \returns true if the rotations are equal within the tolerance
   */
  template<typename OtherDerived_>
  bool isNear(const RotationBase<OtherDerived_,Usage_>& other, typename internal::get_scalar<Derived_>::Scalar tol) const {
    const typename internal::get_scalar<Derived_>::Scalar angle = this->getDisparityAngle(other);
    return (angle <= tol);
  }

  /*! \brief Rotates a vector or a matrix column-wise.
   *  \returns the rotated vector or matrix
   */
  template <typename internal::get_matrix3X<Derived_>::IndexType Cols>
  typename internal::get_matrix3X<Derived_>::template Matrix3X<Cols> rotate(const typename internal::get_matrix3X<Derived_>::template Matrix3X<Cols>& matrix) const {
    return internal::RotationTraits<RotationBase<Derived_,Usage_>>::rotate(this->derived(), matrix);
  }


  /*! \brief Rotates a vector or matrix in reverse.
   *  \returns the reverse rotated vector or matrix
   */
  template <typename internal::get_matrix3X<Derived_>::IndexType Cols>
  typename internal::get_matrix3X<Derived_>::template Matrix3X<Cols> inverseRotate(const typename internal::get_matrix3X<Derived_>::template Matrix3X<Cols>& matrix) const {
    return internal::RotationTraits<RotationBase<Derived_,Usage_>>::rotate(this->derived().inverted(), matrix); // todo: may be optimized
  }

  /*! \brief Rotates a vector.
   *  \returns the rotated vector or matrix
   */
  template <typename Vector_>
  Vector_ rotate(const Vector_& vector) const {
    return internal::RotationTraits<RotationBase<Derived_,Usage_>>::rotate(this->derived(), vector);
  }

  /*! \brief Rotates a vector in reverse.
   *  \returns the reverse rotated vector or matrix
   */
  template <typename Vector_>
  Vector_ inverseRotate(const Vector_& vector) const {
    return internal::RotationTraits<RotationBase<Derived_,Usage_>>::rotate(this->derived().inverted(), vector);
  }

  /*! \brief Sets the rotation using an exponential map @todo avoid altering the rotation
   * \param vector  Eigen::Matrix<Scalar 3, 1>
   * \return  reference to modified rotation
   */
  Derived_ exponentialMap(const typename internal::get_matrix3X<Derived_>::template Matrix3X<1>& vector)  {
   return internal::MapTraits<RotationBase<Derived_,Usage_>>::set_exponential_map(vector);
  }

  /*! \brief Gets the logarithmic map from the rotation
   * \returns vector  Eigen::Matrix<Scalar 3, 1>
   */
  typename internal::get_matrix3X<Derived_>::template Matrix3X<1> logarithmicMap() const {
    return internal::MapTraits<RotationBase<Derived_,Usage_>>::get_logarithmic_map(this->derived());
  }

  /*! \brief Applies the box minus operation
   * \returns vector  Eigen::Matrix<Scalar 3, 1>
   */
  template<typename OtherDerived_>
  typename internal::get_matrix3X<Derived_>::template Matrix3X<1> boxMinus(const RotationBase<OtherDerived_,Usage_>& other) const {
    return internal::BoxOperationTraits<RotationBase<Derived_,Usage_>, RotationBase<OtherDerived_,Usage_>>::box_minus(this->derived(), other.derived());
  }
  /*! \brief Applies the box plus operation
   * \returns rotation
   */
  Derived_ boxPlus(const typename internal::get_matrix3X<Derived_>::template Matrix3X<1>& vector) const {
    return internal::BoxOperationTraits<RotationBase<Derived_,Usage_>, RotationBase<Derived_,Usage_>>::box_plus(this->derived(), vector);
  }



//  /*! \brief Rotates a position.
//   *  \returns the rotated position
//   */
//  template <typename Position_>
//  Position_ rotate(const Position_& position) const {
//    return Position_(internal::RotationTraits<RotationBase<Derived_,Usage_>>::rotate(this->derived(), internal::get_position3<Position_>::get_matrix3(position)));
//  }
//  /*! \brief Rotates a position in reverse.
//   *  \returns the reverse rotated position
//   */
//  template <typename Position_>
//  Position_ inverseRotate(const Position_& position) const {
//    return Position_(internal::RotationTraits<RotationBase<Derived_,Usage_>>::rotate(this->derived().inverted(), internal::get_position3<Position_>::get_matrix3(position)));
//  }

  /*! \brief Sets the rotation from two vectors such that v2 = R(v1).
   */
  template <typename Vector_>
  void setFromVectors(const Vector_& v1, const Vector_& v2) {
    internal::SetFromVectorsTraits<RotationBase<Derived_,Usage_>>::setFromVectors(this->derived(), v1, v2);
  }

  /*! \brief Fixes the rotation to get rid of numerical errors (e.g. normalize quaternion).
   */
  void fix() {
    internal::FixingTraits<Derived_>::fix(this->derived());
  }
};



/*! \brief Representation of a generic angle axis rotation
 *  \ingroup rotations
 *  \class AngleAxisBase
 *  This class defines the generic interface for an angle axis rotation.
 *  \tparam Implementation_ the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation_, enum RotationUsage Usage_>
class AngleAxisBase : public RotationBase<Implementation_, Usage_> {

};

/*! \brief Representation of a generic rotation vector
 *  \ingroup rotations
 *  \class RotationVectorBase
 *
 *  This class defines the generic interface for a rotation vector, which has three paramters.
 *  \see AngleAxisBase for a representation with four parameters.
 *  \tparam Implementation_ the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation_, enum RotationUsage Usage_>
class RotationVectorBase : public RotationBase<Implementation_, Usage_> {

};

/*! \brief Representation of a generic quaternion rotation
 *  \ingroup rotations
 *  \class RotationQuaternionBase
 *  This class defines the generic interface for a quaternion rotation.
 *  \tparam Implementation_ the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation_, enum RotationUsage Usage_>
class RotationQuaternionBase : public RotationBase<Implementation_, Usage_> {

};

/*! \brief Representation of a generic matrix rotation
 *  \ingroup rotations
 *  \class RotationMatrixBase
 *  This class defines the generic interface for a matrix rotation.
 *  \tparam Implementation_ the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 */
template<typename Implementation_, enum RotationUsage Usage_>
class RotationMatrixBase : public RotationBase<Implementation_, Usage_> {

};

/*! \class EulerAnglesBase
 *  \brief Representation of a generic Euler angles rotation
 *
 *  This class defines the generic interface for a Euler angles rotation.
 *  The implementation of a representation of a rotation based on Euler angles
 *  needs to define three angles and the axis-convention that can be
 *  x-y-z, y-z-x, z-x-y, x-z-y, z-y-x, y-x-z or
 *  z-x-z, x-y-x, y-z-y, z-y-z, x-z-x, y-x-y.
 *
 *  \tparam Implementation_ the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  \ingroup rotations
 */
template<typename Implementation_, enum RotationUsage Usage_>
class EulerAnglesBase : public RotationBase<Implementation_, Usage_> {

};

/*! \class EulerAnglesXyzBase
 *  \brief Representation of a generic Euler angles x'-y''-z' rotation
 *
 *  This class defines the generic interface for a Euler angles (X,Y',Z'' / roll,pitch,yaw) rotation.
 *
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  \ingroup rotations
 */
template<typename Implementation_, enum RotationUsage Usage_>
class EulerAnglesXyzBase : public EulerAnglesBase<Implementation_, Usage_> {

};

/*! \class EulerAnglesZyxBase
 *  \brief Representation of a generic Euler angles z-y'-x'' rotation
 *
 *  This class defines the generic interface for a Euler angles (Z-Y'-X'' / yaw-pitch-roll) rotation.
 *
 *  \tparam Implementation the derived class that should implement the rotation
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  \ingroup rotations
 */
template<typename Implementation_, enum RotationUsage Usage_>
class EulerAnglesZyxBase : public EulerAnglesBase<Implementation_, Usage_> {

};


namespace internal {


template<typename Derived_>
class UsageConversionTraits<Derived_,RotationUsage::PASSIVE> {
 public:
  inline static typename get_other_usage<Derived_>::OtherUsage getActive(const RotationBase<Derived_,RotationUsage::PASSIVE>& in) {
    return typename get_other_usage<Derived_>::OtherUsage(in.derived().inverted().toImplementation());
  }

  // getPassive() does not exist (on purpose)
};

template<typename Derived_>
class UsageConversionTraits<Derived_,RotationUsage::ACTIVE> {
 public:
  inline static typename get_other_usage<Derived_>::OtherUsage getPassive(const RotationBase<Derived_,RotationUsage::ACTIVE>& in) {
    return typename get_other_usage<Derived_>::OtherUsage(in.derived().inverted().toImplementation());
  }

  // getActive() does not exist (on purpose)
};



} // namespace internal
} // namespace rotations
} // namespace kindr

#endif /* KINDR_ROTATIONS_ROTATION_BASE_HPP_ */
