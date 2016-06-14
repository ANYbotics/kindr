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

#include <cmath>

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationBase.hpp"

namespace kindr {


/*! \class AngleAxis
 * \brief Implementation of an angle axis rotation based on Eigen::AngleAxis
 *
 *  The following two typedefs are provided for convenience:
 *   - \ref AngleAxisAD "AngleAxisAD" for active rotation and primitive type double
 *   - \ref AngleAxisAF "AngleAxisAF" for active rotation and primitive type float
 *   - \ref AngleAxisPD "AngleAxisPD" for passive rotation and primitive type double
 *   - \ref AngleAxisPF "AngleAxisPF" for passive rotation and primitive type float
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *
 *  \ingroup rotations
 */
template<typename PrimType_>
class AngleAxis : public RotationBase<AngleAxis<PrimType_>> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::AngleAxis<PrimType_> Base;

  /*! Data container
   */
  Base angleAxis_;

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

  /*! \brief All four parameters stored in a vector [angle; axis]
   */
  typedef Eigen::Matrix<PrimType_, 4, 1> Vector4;

  /*! \brief Default constructor using identity rotation.
   */
  AngleAxis()
    : angleAxis_(Base::Identity()) {
  }

  /*! \brief Constructor using four scalars.
   *  In debug mode, an assertion is thrown if the rotation vector has not unit length.
   *  \param angle     rotation angle
   *  \param v1      first entry of the rotation axis vector
   *  \param v2      second entry of the rotation axis vector
   *  \param v3      third entry of the rotation axis vector
   */
  AngleAxis(Scalar angle, Scalar v1, Scalar v2, Scalar v3)
    : angleAxis_(angle,Vector3(v1,v2,v3)) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using angle and axis.
   * In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param angle   rotation angle
   * \param axis     rotation vector with unit length (Eigen vector)
   */
  AngleAxis(Scalar angle, const Vector3& axis)
    : angleAxis_(angle,axis) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }


  /*! \brief Constructor using a 4x1matrix.
   * In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param vector     4x1-matrix with [angle; axis]
   */
  AngleAxis(const Vector4& vector)
    : angleAxis_(vector(0),vector.template block<3,1>(1,0)) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using Eigen::AngleAxis.
   *  In debug mode, an assertion is thrown if the rotation vector has not unit length.
   *  \param other   Eigen::AngleAxis<PrimType_>
   */
  explicit AngleAxis(const Base& other) // explicit on purpose
    : angleAxis_(other) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit AngleAxis(const RotationBase<OtherDerived_>& other)
    : angleAxis_(internal::ConversionTraits<AngleAxis, OtherDerived_>::convert(other.derived()).toImplementation()) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  AngleAxis& operator =(const RotationBase<OtherDerived_>& other) {
    this->toImplementation() = internal::ConversionTraits<AngleAxis, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  AngleAxis& operator ()(const RotationBase<OtherDerived_>& other) {
    this->toImplementation() = internal::ConversionTraits<AngleAxis, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  AngleAxis inverted() const {
    Base inverse = this->toImplementation().inverse();
    inverse.angle() = -inverse.angle();
    inverse.axis() = -inverse.axis();
    return AngleAxis(inverse);
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  AngleAxis& invert() {
    *this = this->inverted();
    return *this;
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(angleAxis_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return static_cast<const Implementation&>(angleAxis_);
  }

  /*! \brief Returns the rotation angle.
   *  \returns rotation angle (scalar)
   */
  inline Scalar angle() const {
    return angleAxis_.angle();
  }

  /*! \brief Sets the rotation angle.
   */
  inline void setAngle(Scalar angle) {
    angleAxis_.angle() = angle;
  }

  /*! \brief Returns the rotation axis.
   *  \returns rotation axis (vector)
   */
  inline const Vector3& axis() const {
    return angleAxis_.axis();
  }

  /*! \brief Sets the rotation axis.
   */
  inline void setAxis(const Vector3& axis) {
    angleAxis_.axis() = axis;
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Sets the rotation axis.
   */
  inline void setAxis(Scalar v1, Scalar v2, Scalar v3) {
    angleAxis_.axis() = Vector3(v1,v2,v3);
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input rotation axis has not unit length.");
  }

  /*! \brief Sets angle-axis from a 4x1-matrix
   */
  inline void setVector(const Vector4& vector) {
    this->setAngle(vector(0));
    this->setAxis(vector.template block<3,1>(1,0));
  }

  /*! \returns the angle and axis in a 4x1 vector [angle; axis].
   */
  inline Vector4 vector() const {
    Vector4 vector;
    vector(0) = angle();
    vector.template block<3,1>(1,0) = axis();
    return vector;
  }


  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  AngleAxis& setIdentity() {
    this->setAngle(static_cast<Scalar>(0));
    this->setAxis(static_cast<Scalar>(1), static_cast<Scalar>(0), static_cast<Scalar>(0));
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
  using RotationBase<AngleAxis<PrimType_>>::operator*;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const AngleAxis& a) {
    out << a.angle() << ", " << a.axis().transpose();
    return out;
  }
};

//! \brief Active angle axis rotation with double primitive type
typedef AngleAxis<double>  AngleAxisPD;
//! \brief Active angle axis rotation with float primitive type
typedef AngleAxis<float>  AngleAxisPF;
//! \brief Passive angle axis rotation with double primitive type
typedef AngleAxis<double> AngleAxisD;
//! \brief Passive angle axis rotation with float primitive type
typedef AngleAxis<float> AngleAxisF;



namespace internal {

template<typename PrimType_>
class get_scalar<AngleAxis<PrimType_>> {
 public:
  typedef PrimType_ Scalar;
};

template<typename PrimType_>
class get_matrix3X<AngleAxis<PrimType_>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<AngleAxis<DestPrimType_>, AngleAxis<SourcePrimType_>> {
 public:
  inline static AngleAxis<DestPrimType_> convert(const AngleAxis<SourcePrimType_>& a) {
    return AngleAxis<DestPrimType_>(a.toImplementation().template cast<DestPrimType_>());
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<AngleAxis<DestPrimType_>, RotationVector<SourcePrimType_>> {
 public:
  inline static AngleAxis<DestPrimType_> convert(const RotationVector<SourcePrimType_>& rotationVector) {
    typedef typename RotationVector<SourcePrimType_>::Scalar Scalar;

    const RotationVector<DestPrimType_> rv(rotationVector);

    if (rv.toImplementation().norm() < common::internal::NumTraits<Scalar>::dummy_precision()) {
      return AngleAxis<DestPrimType_>();
    }
    return AngleAxis<DestPrimType_>(rv.toImplementation().norm(), rv.toImplementation().normalized());
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<AngleAxis<DestPrimType_>, RotationQuaternion<SourcePrimType_>> {
 public:
  inline static AngleAxis<DestPrimType_> convert(const RotationQuaternion<SourcePrimType_>& q) {
    // bad prescion !
    return AngleAxis<DestPrimType_>(Eigen::AngleAxis<DestPrimType_>(q.toImplementation().template cast<DestPrimType_>()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<AngleAxis<DestPrimType_>, RotationMatrix<SourcePrimType_>> {
 public:
  inline static AngleAxis<DestPrimType_> convert(const RotationMatrix<SourcePrimType_>& rotationMatrix) {
//    return AngleAxis<DestPrimType_>(getAngleAxisFromRotationMatrix<SourcePrimType_, DestPrimType_>(rotationMatrix.toImplementation()));
//    return AngleAxis<DestPrimType_>(internal::getAngleAxisFromRotationMatrix<SourcePrimType_, DestPrimType_>(rotationMatrix.toImplementation()));

//   if (Usage_ == RotationUsage::ACTIVE) {
//     return AngleAxis<DestPrimType_>(internal::getAngleAxisFromRotationMatrix<SourcePrimType_, DestPrimType_>(rotationMatrix.matrix()));
//
//   }
//   if (Usage_ == RotationUsage::PASSIVE) {
//     return AngleAxis<DestPrimType_>(internal::getAngleAxisFromRotationMatrix<SourcePrimType_, DestPrimType_>(rotationMatrix.toImplementation()));
//
//   }
//
    // Bad precision!
   return AngleAxis<DestPrimType_>(Eigen::AngleAxis<DestPrimType_>(rotationMatrix.toImplementation().template cast<DestPrimType_>()));

  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<AngleAxis<DestPrimType_>, EulerAnglesXyz<SourcePrimType_>> {
 public:
  inline static AngleAxis<DestPrimType_> convert(const EulerAnglesXyz<SourcePrimType_>& xyz) {
    // bad precision!
    return AngleAxis<DestPrimType_>(Eigen::AngleAxis<DestPrimType_>(
        Eigen::AngleAxis<DestPrimType_>((DestPrimType_)xyz.toImplementation()(0), Eigen::Matrix<DestPrimType_, 3, 1>::UnitX()) *
        Eigen::AngleAxis<DestPrimType_>((DestPrimType_)xyz.toImplementation()(1), Eigen::Matrix<DestPrimType_, 3, 1>::UnitY()) *
        Eigen::AngleAxis<DestPrimType_>((DestPrimType_)xyz.toImplementation()(2), Eigen::Matrix<DestPrimType_, 3, 1>::UnitZ())));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<AngleAxis<DestPrimType_>, EulerAnglesZyx<SourcePrimType_>> {
 public:
  inline static AngleAxis<DestPrimType_> convert(const EulerAnglesZyx<SourcePrimType_>& zyx) {
    // Bad precision!
    return AngleAxis<DestPrimType_>(Eigen::AngleAxis<DestPrimType_>(
      Eigen::AngleAxis<DestPrimType_>((DestPrimType_)zyx.toImplementation()(0), Eigen::Matrix<DestPrimType_, 3, 1>::UnitZ()) *
      Eigen::AngleAxis<DestPrimType_>((DestPrimType_)zyx.toImplementation()(1), Eigen::Matrix<DestPrimType_, 3, 1>::UnitY()) *
      Eigen::AngleAxis<DestPrimType_>((DestPrimType_)zyx.toImplementation()(2), Eigen::Matrix<DestPrimType_, 3, 1>::UnitX())));
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Multiplication Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */



/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Comparison Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename PrimType_>
class ComparisonTraits<AngleAxis<PrimType_>, AngleAxis<PrimType_>> {
 public:
  inline static bool isEqual(const AngleAxis<PrimType_>& a, const AngleAxis<PrimType_>& b){
    const double tolPercent = 0.01;
    return common::eigen::compareRelative(a.angle(), b.angle(), tolPercent) &&
        common::eigen::compareRelative(a.axis().x(), b.axis().x(), tolPercent) &&
        common::eigen::compareRelative(a.axis().y(), b.axis().y(), tolPercent) &&
        common::eigen::compareRelative(a.axis().z(), b.axis().z(), tolPercent);
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Fixing Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename PrimType_>
class FixingTraits<AngleAxis<PrimType_>> {
 public:
  inline static void fix(AngleAxis<PrimType_>& aa) {
    aa.setAxis(aa.axis().normalized());
  }
};

} // namespace internal
} // namespace kindr
