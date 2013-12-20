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

#ifndef KINDR_ROTATIONS_EIGEN_ANGLEAXIS_HPP_
#define KINDR_ROTATIONS_EIGEN_ANGLEAXIS_HPP_

#include <cmath>

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationBase.hpp"
#include "kindr/rotations/eigen/RotationEigenFunctions.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {

/*! \class AngleAxis
 * \brief Implementation of an angle axis rotation based on Eigen::AngleAxis
 *
 *  The following two typedefs are provided for convenience:
 *   - \ref eigen_impl::AngleAxisAD "AngleAxisAD" for active rotation and primitive type double
 *   - \ref eigen_impl::AngleAxisAF "AngleAxisAF" for active rotation and primitive type float
 *   - \ref eigen_impl::AngleAxisPD "AngleAxisPD" for passive rotation and primitive type double
 *   - \ref eigen_impl::AngleAxisPF "AngleAxisPF" for passive rotation and primitive type float
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *
 *  \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class AngleAxis : public AngleAxisBase<AngleAxis<PrimType_, Usage_>, Usage_>, private Eigen::AngleAxis<PrimType_> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::AngleAxis<PrimType_> Base;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;
  /*! \brief The axis type is a 3D vector.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Vector3;

  /*! \brief Default constructor using identity rotation.
   */
  AngleAxis()
    : Base(Base::Identity()) {
  }

  /*! \brief Constructor using four scalars.
   *  In debug mode, an assertion is thrown if the rotation vector has not unit length.
   *  \param angle     rotation angle
   *  \param v1      first entry of the rotation axis vector
   *  \param v2      second entry of the rotation axis vector
   *  \param v3      third entry of the rotation axis vector
   */
  AngleAxis(Scalar angle, Scalar v1, Scalar v2, Scalar v3) {
    if(Usage_ == RotationUsage::ACTIVE) {
      Base::angle() = angle;
      Base::axis() = Vector3(v1,v2,v3);
    } else if(Usage_ == RotationUsage::PASSIVE) {
      Base::angle() = -angle;
      Base::axis() = Vector3(v1,v2,v3);
    }
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using angle and axis.
   * In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param angle   rotation angle
   * \param vector     rotation vector with unit length (Eigen vector)
   */
  AngleAxis(Scalar angle, const Vector3& vector) {
    if(Usage_ == RotationUsage::ACTIVE) {
      this->toStoredImplementation() = Base(angle,vector);
    } else if(Usage_ == RotationUsage::PASSIVE) {
      this->toStoredImplementation() = Base(-angle,vector);
    }
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using Eigen::AngleAxis.
   *  In debug mode, an assertion is thrown if the rotation vector has not unit length.
   *  \param other   Eigen::AngleAxis<PrimType_>
   */
  explicit AngleAxis(const Base& other) // explicit on purpose
    : Base(other) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit AngleAxis(const RotationBase<OtherDerived_, Usage_>& other)
    : Base(internal::ConversionTraits<AngleAxis, OtherDerived_>::convert(other.derived())) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  AngleAxis& operator =(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toStoredImplementation() = internal::ConversionTraits<AngleAxis, OtherDerived_>::convert(other.derived()).toStoredImplementation();
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  AngleAxis& operator ()(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toStoredImplementation() = internal::ConversionTraits<AngleAxis, OtherDerived_>::convert(other.derived()).toStoredImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  AngleAxis inverted() const {
    return AngleAxis(Base::inverse());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  AngleAxis& invert() {
    *this = AngleAxis(Base::inverse());
    return *this;
  }

  /*! \brief Returns the type used for the implementation.
   *  \returns the type used for the implementation
   */
  Implementation toImplementation() const {
    if(Usage_ == RotationUsage::ACTIVE) {
      return Implementation(this->angle(),this->axis());
    } else if(Usage_ == RotationUsage::PASSIVE) {
      return Implementation(-this->angle(),this->axis());
    }
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation& toStoredImplementation() {
    return static_cast<Implementation&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation& toStoredImplementation() const {
    return static_cast<const Implementation&>(*this);
  }

  /*! \brief Returns the rotation angle.
   *  \returns rotation angle (scalar)
   */
  inline Scalar angle() const {
    if(Usage_ == RotationUsage::ACTIVE) {
      return Base::angle();
    } else if(Usage_ == RotationUsage::PASSIVE) {
      return -Base::angle();
    }
  }

  /*! \brief Sets the rotation angle.
   */
  inline void setAngle(Scalar angle) {
    if(Usage_ == RotationUsage::ACTIVE) {
      Base::angle() = angle;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      Base::angle() = -angle;
    }
  }

  /*! \brief Returns the rotation axis.
   *  \returns rotation axis (vector)
   */
  inline const Vector3& axis() const {
    return Base::axis();
  }

  /*! \brief Sets the rotation axis.
   */
  inline void setAxis(const Vector3 & axis) {
    Base::axis() = axis;
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  AngleAxis& setIdentity() {
    this->angle() = static_cast<Scalar>(0);
    this->axis() << static_cast<Scalar>(1), static_cast<Scalar>(0), static_cast<Scalar>(0);
    return *this;
  }

  /*! \brief Returns a unique angle axis rotation with angle in [0,pi].
   *  This function is used to compare different rotations.
   *  \returns copy of the angle axis rotation which is unique
   */
  AngleAxis getUnique() const {
    AngleAxis aa(kindr::common::floatingPointModulo(angle()+M_PI,2*M_PI)-M_PI, axis()); // first wraps angle into [-pi,pi)
    if(aa.angle() > 0)  {
      return aa;
    } else if(aa.angle() < 0) {
      if(aa.angle() != -M_PI) {
        return AngleAxis(-aa.angle(),-aa.axis());
      } else { // angle == -pi, so axis must be viewed further, because -pi,axis does the same as -pi,-axis

        if(aa.axis()[0] < 0) {
          return AngleAxis(-aa.angle(),-aa.axis());
        } else if(aa.axis()[0] > 0) {
          return AngleAxis(-aa.angle(),aa.axis());
        } else { // v1 == 0

          if(aa.axis()[1] < 0) {
            return AngleAxis(-aa.angle(),-aa.axis());
          } else if(aa.axis()[1] > 0) {
            return AngleAxis(-aa.angle(),aa.axis());
          } else { // v2 == 0

            if(aa.axis()[2] < 0) { // v3 must be -1 or 1
              return AngleAxis(-aa.angle(),-aa.axis());
            } else  {
              return AngleAxis(-aa.angle(),aa.axis());
            }
          }
        }
      }
    } else { // angle == 0
      return AngleAxis();
    }
  }

  /*! \brief Modifies the angle axis rotation such that the lies angle in [0,pi).
   *  \returns reference
   */
  AngleAxis& setUnique() {
    *this = getUnique();
    return *this;
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using AngleAxisBase<AngleAxis<PrimType_, Usage_>, Usage_>::operator*;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const AngleAxis& a) {
    out << a.angle() << ", " << a.axis().transpose();
    return out;
  }
};

//! \brief Active angle axis rotation with double primitive type
typedef AngleAxis<double, RotationUsage::ACTIVE>  AngleAxisAD;
//! \brief Active angle axis rotation with float primitive type
typedef AngleAxis<float,  RotationUsage::ACTIVE>  AngleAxisAF;
//! \brief Passive angle axis rotation with double primitive type
typedef AngleAxis<double, RotationUsage::PASSIVE> AngleAxisPD;
//! \brief Passive angle axis rotation with float primitive type
typedef AngleAxis<float,  RotationUsage::PASSIVE> AngleAxisPF;



} // namespace eigen_impl


namespace internal {


template<typename PrimType_, enum RotationUsage Usage_>
class get_matrix3X<eigen_impl::AngleAxis<PrimType_, Usage_>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};


template<typename PrimType_>
class get_other_usage<eigen_impl::AngleAxis<PrimType_, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_impl::AngleAxis<PrimType_, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::AngleAxis<PrimType_, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_impl::AngleAxis<PrimType_, RotationUsage::ACTIVE> OtherUsage;
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::AngleAxis<DestPrimType_, Usage_>, eigen_impl::AngleAxis<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::AngleAxis<DestPrimType_, Usage_> convert(const eigen_impl::AngleAxis<SourcePrimType_, Usage_>& a) {
    return eigen_impl::AngleAxis<DestPrimType_, Usage_>(a.toStoredImplementation().template cast<DestPrimType_>());
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::AngleAxis<DestPrimType_, Usage_>, eigen_impl::RotationVector<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::AngleAxis<DestPrimType_, Usage_> convert(const eigen_impl::RotationVector<SourcePrimType_, Usage_>& rv) {
    return eigen_impl::AngleAxis<DestPrimType_, Usage_>(rv.toStoredImplementation().norm(), rv.toStoredImplementation().normalized());
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::AngleAxis<DestPrimType_, Usage_>, eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::AngleAxis<DestPrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>& q) {
    return eigen_impl::AngleAxis<DestPrimType_, Usage_>(eigen_impl::getAngleAxisFromQuaternion<SourcePrimType_, DestPrimType_>(q.toStoredImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::AngleAxis<DestPrimType_, Usage_>, eigen_impl::RotationMatrix<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::AngleAxis<DestPrimType_, Usage_> convert(const eigen_impl::RotationMatrix<SourcePrimType_, Usage_>& R) {
    return eigen_impl::AngleAxis<DestPrimType_, Usage_>(eigen_impl::getAngleAxisFromRotationMatrix<SourcePrimType_, DestPrimType_>(R.toStoredImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::AngleAxis<DestPrimType_, Usage_>, eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::AngleAxis<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>& xyz) {
    return eigen_impl::AngleAxis<DestPrimType_, Usage_>(eigen_impl::getAngleAxisFromRpy<SourcePrimType_, DestPrimType_>(xyz.toStoredImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::AngleAxis<DestPrimType_, Usage_>, eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::AngleAxis<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>& zyx) {
    return eigen_impl::AngleAxis<DestPrimType_, Usage_>(eigen_impl::getAngleAxisFromYpr<SourcePrimType_, DestPrimType_>(zyx.toStoredImplementation()));
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Multiplication Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename PrimType_, enum RotationUsage Usage_>
class RotationTraits<eigen_impl::AngleAxis<PrimType_, Usage_>> {
 public:
  template<typename get_matrix3X<eigen_impl::AngleAxis<PrimType_, Usage_>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_impl::AngleAxis<PrimType_, Usage_>>::template Matrix3X<Cols> rotate(const eigen_impl::AngleAxis<PrimType_, Usage_>& aa, const typename get_matrix3X<eigen_impl::AngleAxis<PrimType_, Usage_>>::template Matrix3X<Cols>& m){
    return eigen_impl::RotationMatrix<PrimType_, Usage_>(aa).toStoredImplementation() * m;
  }
};

///* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// * Comparison Traits
// * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
//template<typename PrimType_, enum RotationUsage Usage_>
//class ComparisonTraits<eigen_impl::AngleAxis<PrimType_, Usage_>> {
// public:
//  inline static bool isEqual(const eigen_impl::AngleAxis<PrimType_, Usage_>& a, const eigen_impl::AngleAxis<PrimType_, Usage_>& b){
//    return a.toStoredImplementation().angle() ==  b.toStoredImplementation().angle() &&
//           a.toStoredImplementation().axis()  ==  b.toStoredImplementation().axis();
//  }
//};

} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_EIGEN_ANGLEAXIS_HPP_ */
