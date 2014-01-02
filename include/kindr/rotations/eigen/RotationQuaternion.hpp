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

#ifndef KINDR_ROTATIONS_EIGEN_ROTATIONQUATERNION_HPP_
#define KINDR_ROTATIONS_EIGEN_ROTATIONQUATERNION_HPP_

#include <cmath>

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/quaternions/QuaternionEigen.hpp"
#include "kindr/rotations/RotationBase.hpp"
#include "kindr/rotations/eigen/RotationEigenFunctions.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {



/*!  \class RotationQuaternion
 *  \brief Implementation of quaternion rotation based on Eigen::Quaternion
 *
 *  The following four typedefs are provided for convenience:
 *   - \ref eigen_impl::RotationQuaternionAD "RotationQuaternionAD" for active rotation and primitive type double
 *   - \ref eigen_impl::RotationQuaternionAF "RotationQuaternionAF" for active rotation and primitive type float
 *   - \ref eigen_impl::RotationQuaternionPD "RotationQuaternionPD" for passive rotation and primitive type double
 *   - \ref eigen_impl::RotationQuaternionPF "RotationQuaternionPF" for passive rotation and primitive type float
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *
 *  \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class RotationQuaternion : public RotationQuaternionBase<RotationQuaternion<PrimType_, Usage_>, Usage_> {
 private:
  /*! \brief The base type.
   */
  typedef quaternions::eigen_impl::UnitQuaternion<PrimType_> Base;

  /*! \brief The data container
   */
  Base rotationQuaternion_;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef typename Base::Implementation Implementation;

  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  //! the imaginary type, i.e., Eigen::Quaternion<>
  typedef Eigen::Matrix<PrimType_,3,1> Imaginary;

  /*! \brief Default constructor using identity rotation.
   */
  RotationQuaternion()
    : rotationQuaternion_(Implementation::Identity()) {
  }

  /*! \brief Constructor using four scalars.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param w     first entry of the quaternion = cos(phi/2)
   *  \param x     second entry of the quaternion = n1*sin(phi/2)
   *  \param y     third entry of the quaternion = n2*sin(phi/2)
   *  \param z     fourth entry of the quaternion = n3*sin(phi/2)
   */
  RotationQuaternion(Scalar w, Scalar x, Scalar y, Scalar z) {
    if(Usage_ == RotationUsage::ACTIVE) {
      rotationQuaternion_.w() = w;
      rotationQuaternion_.x() = x;
      rotationQuaternion_.y() = y;
      rotationQuaternion_.z() = z;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      rotationQuaternion_.w() = w;
      rotationQuaternion_.x() = -x;
      rotationQuaternion_.y() = -y;
      rotationQuaternion_.z() = -z;
    }
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input quaternion has not unit length.");
  }

  RotationQuaternion(const Scalar& real, const Imaginary& imag) {
    if(Usage_ == RotationUsage::ACTIVE) {
      rotationQuaternion_.w() = real;
      rotationQuaternion_.x() = imag(0);
      rotationQuaternion_.y() = imag(1);
      rotationQuaternion_.z() = imag(2);
    } else if(Usage_ == RotationUsage::PASSIVE) {
      rotationQuaternion_.w() = real;
      rotationQuaternion_.x() = -imag(0);
      rotationQuaternion_.y() = -imag(1);
      rotationQuaternion_.z() = -imag(2);
    }
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using Eigen::Quaternion<PrimType_>.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param other   Eigen::Quaternion<PrimType_>
   */
  explicit RotationQuaternion(const Implementation& other) {
    if(Usage_ == RotationUsage::ACTIVE) {
      rotationQuaternion_.w() = other.w();
      rotationQuaternion_.x() = other.x();
      rotationQuaternion_.y() = other.y();
      rotationQuaternion_.z() = other.z();
    } else if(Usage_ == RotationUsage::PASSIVE) {
      rotationQuaternion_.w() = other.w();
      rotationQuaternion_.x() = -other.x();
      rotationQuaternion_.y() = -other.y();
      rotationQuaternion_.z() = -other.z();
    }
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using quaternions::UnitQuaternion.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param other   quaternions::UnitQuaternion
   */
  explicit RotationQuaternion(const Base& other) {
    if(Usage_ == RotationUsage::ACTIVE) {
      rotationQuaternion_.w() = other.w();
      rotationQuaternion_.x() = other.x();
      rotationQuaternion_.y() = other.y();
      rotationQuaternion_.z() = other.z();
    } else if(Usage_ == RotationUsage::PASSIVE) {
      rotationQuaternion_.w() = other.w();
      rotationQuaternion_.x() = -other.x();
      rotationQuaternion_.y() = -other.y();
      rotationQuaternion_.z() = -other.z();
    }
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit RotationQuaternion(const RotationBase<OtherDerived_, Usage_>& other)
    : rotationQuaternion_(internal::ConversionTraits<RotationQuaternion, OtherDerived_>::convert(other.derived()).toStoredImplementation()) {
  }

  inline Scalar w() const {
    return rotationQuaternion_.w();
  }

  inline Scalar x() const {
    if(Usage_ == RotationUsage::ACTIVE) {
      return rotationQuaternion_.x();
    } else if(Usage_ == RotationUsage::PASSIVE) {
      return -rotationQuaternion_.x();
    }
  }

  inline Scalar y() const {
    if(Usage_ == RotationUsage::ACTIVE) {
      return rotationQuaternion_.y();
    } else if(Usage_ == RotationUsage::PASSIVE) {
      return -rotationQuaternion_.y();
    }
  }

  inline Scalar z() const {
    if(Usage_ == RotationUsage::ACTIVE) {
      return rotationQuaternion_.z();
    } else if(Usage_ == RotationUsage::PASSIVE) {
      return -rotationQuaternion_.z();
    }
  }

  inline void setW(Scalar w) { // todo: attention: no assertion for unitquaternions
    rotationQuaternion_.w() = w;
  }

  inline void setX(Scalar x) {
    if(Usage_ == RotationUsage::ACTIVE) {
      rotationQuaternion_.x() = x;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      rotationQuaternion_.x() = -x;
    }
  }

  inline void setY(Scalar y) {
    if(Usage_ == RotationUsage::ACTIVE) {
      rotationQuaternion_.y() = y;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      rotationQuaternion_.y() = -y;
    }
  }

  inline void setZ(Scalar z) {
    if(Usage_ == RotationUsage::ACTIVE) {
      rotationQuaternion_.z() = z;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      rotationQuaternion_.z() = -z;
    }
  }


  inline Scalar real() const {
    return this->toUnitQuaternion().real();
  }

  inline Imaginary imaginary() const {
    return this->toUnitQuaternion().imaginary();
  }

  inline void setReal(Scalar real) {
    rotationQuaternion_.w() = real;
  }

  inline void setImaginary(Imaginary imag) {
    if(Usage_ == RotationUsage::ACTIVE) {
      rotationQuaternion_.x() = imag(0);
      rotationQuaternion_.y() = imag(1);
      rotationQuaternion_.z() = imag(2);
    } else if(Usage_ == RotationUsage::PASSIVE) {
      rotationQuaternion_.x() = -imag(0);
      rotationQuaternion_.y() = -imag(1);
      rotationQuaternion_.z() = -imag(2);
    }
  }

  Base toUnitQuaternion() const {
    return Base(this->w(),this->x(),this->y(),this->z());
  }

  Implementation toImplementation() const {
    return Implementation(this->w(),this->x(),this->y(),this->z());
  }

  Base& toStoredUnitQuaternion() {
     return static_cast<Base&>(rotationQuaternion_);
   }

  const Base& toStoredUnitQuaternion() const {
     return static_cast<const Base&>(rotationQuaternion_);
   }

  Implementation& toStoredImplementation()  {
    return this->toStoredUnitQuaternion().toImplementation();
  }

  const Implementation& toStoredImplementation() const {
    return this->toStoredUnitQuaternion().toImplementation();
  }

  /*! \brief Assignment operator using a UnitQuaternion.
   *  \param quat   UnitQuaternion
   *  \returns reference
   */
  template<typename PrimTypeIn_>
  RotationQuaternion& operator =(const quaternions::eigen_impl::UnitQuaternion<PrimTypeIn_>& quat) {
    if(Usage_ == RotationUsage::ACTIVE) {
      this->toStoredImplementation() = Implementation(quat.w(),quat.x(),quat.y(),quat.z());
    } else if(Usage_ == RotationUsage::PASSIVE) {
      this->toStoredImplementation() = Implementation(quat.w(),-quat.x(),-quat.y(),-quat.z());
    }
    return *this;
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationQuaternion& operator =(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toStoredImplementation() = internal::ConversionTraits<RotationQuaternion, OtherDerived_>::convert(other.derived()).toStoredImplementation();
    return *this;
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename PrimTypeIn_>
  RotationQuaternion& operator =(const RotationQuaternion<PrimTypeIn_, Usage_>& other) {
    this->toStoredImplementation() = other.toStoredImplementation().template cast<PrimType_>();
    return *this;
  }

  /*! \brief Bracket operator which assigns a UnitQuaternion to the RotationQuaternion.
   *  \param quat   UnitQuaternion
   *  \returns reference
   */
  template<typename PrimTypeIn_>
  RotationQuaternion& operator ()(const quaternions::eigen_impl::UnitQuaternion<PrimTypeIn_>& quat) {
    if(Usage_ == RotationUsage::ACTIVE) {
      this->toStoredImplementation() = Implementation(quat.w(),quat.x(),quat.y(),quat.z());
    } else if(Usage_ == RotationUsage::PASSIVE) {
      this->toStoredImplementation() = Implementation(quat.w(),-quat.x(),-quat.y(),-quat.z());
    }
    return *this;
  }

  /*! \brief Bracket operator which assigns a Quaternion to the RotationQuaternion.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param quat   Quaternion
   *  \returns reference
   */
  template<typename PrimTypeIn_>
  RotationQuaternion& operator ()(const quaternions::eigen_impl::Quaternion<PrimTypeIn_>& quat) {
    if(Usage_ == RotationUsage::ACTIVE) {
      this->toStoredImplementation() = Implementation(quat.w(),quat.x(),quat.y(),quat.z());
    } else if(Usage_ == RotationUsage::PASSIVE) {
      this->toStoredImplementation() = Implementation(quat.w(),-quat.x(),-quat.y(),-quat.z());
    }
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input quaternion has not unit length.");
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationQuaternion& operator ()(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toStoredImplementation() = internal::ConversionTraits<RotationQuaternion, OtherDerived_>::convert(other.derived()).toStoredImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  RotationQuaternion inverted() const {
    return RotationQuaternion(this->toUnitQuaternion().inverted());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  RotationQuaternion& invert() {
    *this = inverted();
    return *this;
  }

  /*! \brief Returns the conjugated of the quaternion.
   *  \returns conjugated of the quaternion
   */
  RotationQuaternion conjugated() const {
    return RotationQuaternion(this->toUnitQuaternion().conjugated());
  }

  /*! \brief Conjugates of the quaternion.
   *  \returns reference
   */
  RotationQuaternion& conjugate() {
    *this = conjugated();
    return *this;
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  RotationQuaternion& setIdentity() {
    rotationQuaternion_.w() = static_cast<Scalar>(1);
    rotationQuaternion_.x() = static_cast<Scalar>(0);
    rotationQuaternion_.y() = static_cast<Scalar>(0);
    rotationQuaternion_.z() = static_cast<Scalar>(0);
    return *this;
  }

  /*! \brief Returns a unique quaternion rotation with w > 0.
   *  This function is used to compare different rotations.
   *  \returns copy of the quaternion rotation which is unique
   */
  RotationQuaternion getUnique() const {
    if(this->w() > 0) {
      return *this;
    } else if (this->w() < 0){
      return RotationQuaternion(-this->w(),-this->x(),-this->y(),-this->z());
    } else { // w == 0

      if(this->x() > 0) {
        return *this;
      } else if (this->x() < 0){
        return RotationQuaternion(-this->w(),-this->x(),-this->y(),-this->z());
      } else { // x == 0

        if(this->y() > 0) {
          return *this;
        } else if (this->y() < 0){
          return RotationQuaternion(-this->w(),-this->x(),-this->y(),-this->z());
        } else { // y == 0

          if(this->z() > 0) { // z must be either -1 or 1 in this case
            return *this;
          } else {
            return RotationQuaternion(-this->w(),-this->x(),-this->y(),-this->z());
          }
        }
      }
    }
  }

  /*! \brief Modifies the quaternion rotation such that w >= 0.
   *  \returns reference
   */
  RotationQuaternion& setUnique() {
    *this = getUnique();
    return *this;
  }

  /*! \brief Returns the norm of the quaternion.
   *  The RotationQuaternion should always have unit length.
   *  \returns norm of the quaternion
   */
  Scalar norm() {
    return rotationQuaternion_.norm();
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because QuaternionBase provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using RotationQuaternionBase<RotationQuaternion<PrimType_, Usage_>, Usage_> ::operator*;

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because QuaternionBase provides also an operator==.
   *  \returns true if two rotations are similar.
   */
  using RotationQuaternionBase<RotationQuaternion<PrimType_, Usage_>, Usage_> ::operator==;


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationQuaternion& rquat) {
    if(Usage_ == RotationUsage::ACTIVE) {
      out << rquat.toUnitQuaternion();
    } else if(Usage_ == RotationUsage::PASSIVE) {
      out << rquat.inverted().toUnitQuaternion();
    }
    return out;
  }
};

//! \brief Active quaternion rotation with double primitive type
typedef RotationQuaternion<double, RotationUsage::ACTIVE>  RotationQuaternionAD;
//! \brief Active quaternion rotation with float primitive type
typedef RotationQuaternion<float,  RotationUsage::ACTIVE>  RotationQuaternionAF;
//! \brief Passive quaternion rotation with double primitive type
typedef RotationQuaternion<double, RotationUsage::PASSIVE> RotationQuaternionPD;
//! \brief Passive quaternion rotation with float primitive type
typedef RotationQuaternion<float,  RotationUsage::PASSIVE> RotationQuaternionPF;






} // namespace eigen_impl


namespace internal {

template<typename PrimType_, enum RotationUsage Usage_>
class get_scalar<eigen_impl::RotationQuaternion<PrimType_, Usage_>> {
 public:
  typedef PrimType_ Scalar;
};

template<typename PrimType_, enum RotationUsage Usage_>
class get_matrix3X<eigen_impl::RotationQuaternion<PrimType_, Usage_>>{
 public:
  typedef int IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_impl::RotationQuaternion<PrimType_, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::RotationQuaternion<PrimType_, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE> OtherUsage;
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::AngleAxis<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::AngleAxis<SourcePrimType_, Usage_>& aa) {
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::getQuaternionFromAngleAxis<SourcePrimType_, DestPrimType_>(aa.toImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::RotationVector<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::RotationVector<SourcePrimType_, Usage_>& rotationVector) {
    typedef typename eigen_impl::RotationQuaternion<DestPrimType_, Usage_>::Scalar Scalar;
    typedef typename eigen_impl::RotationQuaternion<DestPrimType_, Usage_>::Imaginary Imaginary;
    const Scalar v = rotationVector.toImplementation().norm();
    Scalar real;
    Imaginary imaginary;
    if (v < common::NumTraits<Scalar>::dummy_precision()) {
      real = 1.0;
      imaginary= 0.5*rotationVector.toImplementation().template cast<DestPrimType_>();
    }
    else {
      real = cos(v/2);
      imaginary = sin(v/2)/v*rotationVector.toImplementation().template cast<DestPrimType_>();
    }
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(real, imaginary);
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>& q) {
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(q.toImplementation().template cast<DestPrimType_>());
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::RotationMatrix<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::RotationMatrix<SourcePrimType_, Usage_>& rotationMatrix) {
    if (Usage_ == RotationUsage::ACTIVE) {
      return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::getQuaternionFromRotationMatrix<SourcePrimType_, DestPrimType_>(rotationMatrix.toImplementation()));
    } if (Usage_ == RotationUsage::PASSIVE) {
      return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::getQuaternionFromRotationMatrix<SourcePrimType_, DestPrimType_>(rotationMatrix.toImplementation()).inverse());
    }
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>& xyz) {
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::getQuaternionFromRpy<SourcePrimType_, DestPrimType_>(xyz.toImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>& zyx) {
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::getQuaternionFromYpr<SourcePrimType_, DestPrimType_>(zyx.toImplementation()));
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
template<typename PrimType_, enum RotationUsage Usage_>
class ComparisonTraits<eigen_impl::RotationQuaternion<PrimType_, Usage_>, eigen_impl::RotationQuaternion<PrimType_, Usage_>> {
 public:
   inline static bool isEqual(const eigen_impl::RotationQuaternion<PrimType_, Usage_>& a, const eigen_impl::RotationQuaternion<PrimType_, Usage_>& b){
     return a.toStoredImplementation().w() ==  b.toStoredImplementation().w() &&
            a.toStoredImplementation().x() ==  b.toStoredImplementation().x() &&
            a.toStoredImplementation().y() ==  b.toStoredImplementation().y() &&
            a.toStoredImplementation().z() ==  b.toStoredImplementation().z();
   }

//   inline static bool isNear(const eigen_impl::RotationQuaternion<PrimType_, Usage_>& a, const eigen_impl::RotationQuaternion<PrimType_, Usage_>& b, PrimType_ tol){
//     return fabs(a.toStoredImplementation().w() - b.toStoredImplementation().w()) < tol &&
//            fabs(a.toStoredImplementation().x() - b.toStoredImplementation().x()) < tol &&
//            fabs(a.toStoredImplementation().y() - b.toStoredImplementation().y()) < tol &&
//            fabs(a.toStoredImplementation().z() - b.toStoredImplementation().z()) < tol;
//   }
};

} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_EIGEN_ROTATIONQUATERNION_HPP_ */
