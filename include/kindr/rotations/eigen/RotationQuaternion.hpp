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

  //! quaternion as 4x1 matrix: [w; x; y; z]
  typedef Eigen::Matrix<PrimType_,4,1> Vector4;

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
  RotationQuaternion(Scalar w, Scalar x, Scalar y, Scalar z)
    : rotationQuaternion_(w,x,y,z) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using real and imaginary part.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param real   real part (PrimType_)
   *  \param imag   imaginary part (Eigen::Matrix<PrimType_,3,1>)
   */
  RotationQuaternion(Scalar real, const Imaginary& imag)
    : rotationQuaternion_(real,imag(0),imag(1),imag(2)) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using Eigen::Matrix<PrimType_,4,1>.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param other   Eigen::Matrix<PrimType_,4,1>
   */
  RotationQuaternion(const Vector4 & vec)
    : rotationQuaternion_(vec(0),vec(1),vec(2),vec(3)) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), 1e-4, "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using Eigen::Quaternion<PrimType_>.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param other   Eigen::Quaternion<PrimType_>
   */
  explicit RotationQuaternion(const Implementation& other)
    : rotationQuaternion_(other.w(), other.x(), other.y(), other.z()) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), 1e-4, "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using quaternions::UnitQuaternion.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param other   quaternions::UnitQuaternion
   */
  explicit RotationQuaternion(const Base& other)
    : rotationQuaternion_(other.w(), other.x(), other.y(), other.z()) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, rotationQuaternion_.norm(), static_cast<Scalar>(1), 1e-4, "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit RotationQuaternion(const RotationBase<OtherDerived_, Usage_>& other)
    : rotationQuaternion_(internal::ConversionTraits<RotationQuaternion, OtherDerived_>::convert(other.derived()).toImplementation()) {
  }

  inline Scalar w() const {
    return rotationQuaternion_.w();
  }

  inline Scalar x() const {
    return rotationQuaternion_.x();
  }

  inline Scalar y() const {
    return rotationQuaternion_.y();
  }

  inline Scalar z() const {
    return rotationQuaternion_.z();
  }


  inline Scalar real() const {
    return this->toUnitQuaternion().real();
  }

  inline Imaginary imaginary() const {
    return this->toUnitQuaternion().imaginary();
  }

  inline Vector4 vector() const {
    Vector4 vector4;
    vector4 << w(), x(), y(), z();
    return vector4;
  }

  inline void setValues(Scalar w, Scalar x, Scalar y, Scalar z) {
    rotationQuaternion_.w() = w;
    rotationQuaternion_.x() = x;
    rotationQuaternion_.y() = y;
    rotationQuaternion_.z() = z;
  }

  inline void setParts(Scalar real, const Imaginary& imag) {
    rotationQuaternion_.w() = real;
    rotationQuaternion_.x() = imag(0);
    rotationQuaternion_.y() = imag(1);
    rotationQuaternion_.z() = imag(2);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return toUnitQuaternion().toImplementation();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return toUnitQuaternion().toImplementation();
  }

//  Implementation toImplementation() const {
//    return Implementation(this->w(),this->x(),this->y(),this->z());
//  }

  Base& toUnitQuaternion() {
     return static_cast<Base&>(rotationQuaternion_);
   }

  const Base& toUnitQuaternion() const {
     return static_cast<const Base&>(rotationQuaternion_);
   }

  /*! \brief Assignment operator using a UnitQuaternion.
   *  \param quat   UnitQuaternion
   *  \returns reference
   */
  template<typename PrimTypeIn_>
  RotationQuaternion& operator =(const quaternions::eigen_impl::UnitQuaternion<PrimTypeIn_>& quat) {
    this->toImplementation() = Implementation(quat.w(),quat.x(),quat.y(),quat.z());
    return *this;
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationQuaternion& operator =(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toImplementation() = internal::ConversionTraits<RotationQuaternion, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename PrimTypeIn_>
  RotationQuaternion& operator =(const RotationQuaternion<PrimTypeIn_, Usage_>& other) {
    this->toImplementation() = other.toImplementation().template cast<PrimType_>();
    return *this;
  }

  /*! \brief Bracket operator which assigns a UnitQuaternion to the RotationQuaternion.
   *  \param quat   UnitQuaternion
   *  \returns reference
   */
  template<typename PrimTypeIn_>
  RotationQuaternion& operator ()(const quaternions::eigen_impl::UnitQuaternion<PrimTypeIn_>& quat) {
    this->toImplementation() = Implementation(quat.w(),quat.x(),quat.y(),quat.z());
    return *this;
  }

  /*! \brief Bracket operator which assigns a Quaternion to the RotationQuaternion.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param quat   Quaternion
   *  \returns reference
   */
  template<typename PrimTypeIn_>
  RotationQuaternion& operator ()(const quaternions::eigen_impl::Quaternion<PrimTypeIn_>& quat) {
    this->toImplementation() = Implementation(quat.w(),quat.x(),quat.y(),quat.z());
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input quaternion has not unit length.");
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationQuaternion& operator ()(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toImplementation() = internal::ConversionTraits<RotationQuaternion, OtherDerived_>::convert(other.derived()).toImplementation();
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

  /*! \brief Returns the quaternion matrix Qleft: q*p = Qleft(q)*p
   *  This function can be used to get the derivative of the concatenation with respect to the right quaternion.
   *  \returns the quaternion matrix Qleft
   */
  Eigen::Matrix<PrimType_,4,4> getQuaternionMatrix() {
    Eigen::Matrix<PrimType_,4,4> Qleft;
    if(Usage_ == rotations::RotationUsage::ACTIVE)
    {
      Qleft(0,0) =  w();      Qleft(0,1) = -x();      Qleft(0,2) = -y();      Qleft(0,3) = -z();
      Qleft(1,0) =  x();      Qleft(1,1) =  w();      Qleft(1,2) = -z();      Qleft(1,3) =  y();
      Qleft(2,0) =  y();      Qleft(2,1) =  z();      Qleft(2,2) =  w();      Qleft(2,3) = -x();
      Qleft(3,0) =  z();      Qleft(3,1) = -y();      Qleft(3,2) =  x();      Qleft(3,3) =  w();
    }
    if(Usage_ == rotations::RotationUsage::PASSIVE)
    {
      Qleft(0,0) =  w();      Qleft(0,1) = -x();      Qleft(0,2) = -y();      Qleft(0,3) = -z();
      Qleft(1,0) =  x();      Qleft(1,1) =  w();      Qleft(1,2) =  z();      Qleft(1,3) = -y();
      Qleft(2,0) =  y();      Qleft(2,1) = -z();      Qleft(2,2) =  w();      Qleft(2,3) =  x();
      Qleft(3,0) =  z();      Qleft(3,1) =  y();      Qleft(3,2) = -x();      Qleft(3,3) =  w();
    }
    return Qleft;
  }

  /*! \brief Returns the quaternion matrix Qright: q*p = Qright(p)*q
   *  This function can be used to get the derivative of the concatenation with respect to the left quaternion.
   *  \returns the quaternion matrix Qright
   */
  Eigen::Matrix<PrimType_,4,4> getConjugateQuaternionMatrix() {
    Eigen::Matrix<PrimType_,4,4> Qright;
    if(Usage_ == rotations::RotationUsage::ACTIVE)
    {
      Qright(0,0) =  w();      Qright(0,1) = -x();      Qright(0,2) = -y();      Qright(0,3) = -z();
      Qright(1,0) =  x();      Qright(1,1) =  w();      Qright(1,2) =  z();      Qright(1,3) = -y();
      Qright(2,0) =  y();      Qright(2,1) = -z();      Qright(2,2) =  w();      Qright(2,3) =  x();
      Qright(3,0) =  z();      Qright(3,1) =  y();      Qright(3,2) = -x();      Qright(3,3) =  w();
    }
    if(Usage_ == rotations::RotationUsage::PASSIVE)
    {
      Qright(0,0) =  w();      Qright(0,1) = -x();      Qright(0,2) = -y();      Qright(0,3) = -z();
      Qright(1,0) =  x();      Qright(1,1) =  w();      Qright(1,2) = -z();      Qright(1,3) =  y();
      Qright(2,0) =  y();      Qright(2,1) =  z();      Qright(2,2) =  w();      Qright(2,3) = -x();
      Qright(3,0) =  z();      Qright(3,1) = -y();      Qright(3,2) =  x();      Qright(3,3) =  w();
    }
    return Qright;
  }

  /*! \brief Returns the global quaternion diff matrix H: GlobalAngularVelocity = 2*H*qdiff, qdiff = 0.5*H^T*GlobalAngularVelocity
   *  \returns the global quaternion diff matrix H
   */
  Eigen::Matrix<PrimType_,3,4> getGlobalQuaternionDiffMatrix() {
    Eigen::Matrix<PrimType_,3,4> H;
    if(Usage_ == rotations::RotationUsage::ACTIVE) // x, y, z * -1
    {
      H(0,0) =  -x();      H(0,1) =  w();      H(0,2) =  z();      H(0,3) = -y();
      H(1,0) =  -y();      H(1,1) = -z();      H(1,2) =  w();      H(1,3) =  x();
      H(2,0) =  -z();      H(2,1) =  y();      H(2,2) = -x();      H(2,3) =  w();
    }
    if(Usage_ == rotations::RotationUsage::PASSIVE)
    {
      H(0,0) = -x();      H(0,1) =  w();      H(0,2) = -z();      H(0,3) =  y();
      H(1,0) = -y();      H(1,1) =  z();      H(1,2) =  w();      H(1,3) = -x();
      H(2,0) = -z();      H(2,1) = -y();      H(2,2) =  x();      H(2,3) =  w();
    }
    return H;
  }

  /*! \brief Returns the local quaternion diff matrix HBar: LocalAngularVelocity = 2*HBar*qdiff, qdiff = 0.5*HBar^T*LocalAngularVelocity
   *  \returns the local quaternion diff matrix HBar
   */
  Eigen::Matrix<PrimType_,3,4> getLocalQuaternionDiffMatrix() const {
    Eigen::Matrix<PrimType_,3,4> HBar;
    if(Usage_ == rotations::RotationUsage::ACTIVE) // x, y, z * -1
    {
      HBar(0,0) =  -x();      HBar(0,1) =  w();      HBar(0,2) = -z();      HBar(0,3) =  y();
      HBar(1,0) =  -y();      HBar(1,1) =  z();      HBar(1,2) =  w();      HBar(1,3) = -x();
      HBar(2,0) =  -z();      HBar(2,1) = -y();      HBar(2,2) =  x();      HBar(2,3) =  w();
    }
    if(Usage_ == rotations::RotationUsage::PASSIVE)
    {
      HBar(0,0) = -x();      HBar(0,1) =  w();      HBar(0,2) =  z();      HBar(0,3) = -y();
      HBar(1,0) = -y();      HBar(1,1) = -z();      HBar(1,2) =  w();      HBar(1,3) =  x();
      HBar(2,0) = -z();      HBar(2,1) =  y();      HBar(2,2) = -x();      HBar(2,3) =  w();
    }
    return HBar;
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
    out << rquat.toUnitQuaternion();
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
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::eigen_internal::getQuaternionFromAngleAxis<SourcePrimType_, DestPrimType_>(aa.toImplementation()));
  }
};


template <typename Scalar_ = double>
inline bool isLessThenEpsilons4thRoot(Scalar_ x){
  static const Scalar_ epsilon4thRoot = pow(std::numeric_limits<Scalar_>::epsilon(), 1.0/4.0);
  return x < epsilon4thRoot;
}

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::RotationVector<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::RotationVector<SourcePrimType_, Usage_>& rotationVector) {
    typedef typename eigen_impl::RotationQuaternion<DestPrimType_, Usage_>::Scalar Scalar;
    typedef typename eigen_impl::RotationQuaternion<DestPrimType_, Usage_>::Imaginary Imaginary;
//    const Scalar v = rotationVector.toImplementation().norm();
//    Scalar real;
//    Imaginary imaginary;
//    if (v < common::internal::NumTraits<Scalar>::dummy_precision()) {
//      real = 1.0;
//      imaginary= 0.5*rotationVector.toImplementation().template cast<DestPrimType_>();
//    }
//    else {
//      real = cos(v/2);
//      imaginary = sin(v/2)/v*rotationVector.toImplementation().template cast<DestPrimType_>();
//    }
//    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(real, imaginary);


    Scalar theta = (Scalar)rotationVector.toImplementation().norm();

    // na is 1/theta sin(theta/2)
    double na;
    if(isLessThenEpsilons4thRoot(theta))
    {
        const Scalar one_over_48 = 1.0/48.0;
        na = 0.5 + (theta * theta) * one_over_48;
    }
    else
    {
        na = sin(theta*0.5) / theta;
    }
    Imaginary axis = rotationVector.toImplementation().template cast<Scalar>()*na;
    Scalar ct = cos(theta*0.5);
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(ct, axis[0],axis[1],axis[2]);
//    return Eigen::Vector4d(axis[0],axis[1],axis[2],ct);

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
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::eigen_internal::getQuaternionFromRotationMatrix<SourcePrimType_, DestPrimType_>(rotationMatrix.toImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>& xyz) {
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::eigen_internal::getQuaternionFromRpy<SourcePrimType_, DestPrimType_>(xyz.toImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationQuaternion<DestPrimType_, Usage_>, eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternion<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>& zyx) {
    return eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(eigen_impl::eigen_internal::getQuaternionFromYpr<SourcePrimType_, DestPrimType_>(zyx.toImplementation()));
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
     return a.toImplementation().w() ==  b.toImplementation().w() &&
            a.toImplementation().x() ==  b.toImplementation().x() &&
            a.toImplementation().y() ==  b.toImplementation().y() &&
            a.toImplementation().z() ==  b.toImplementation().z();
   }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Fixing Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename PrimType_, enum RotationUsage Usage_>
class FixingTraits<eigen_impl::RotationQuaternion<PrimType_, Usage_>> {
 public:
  inline static void fix(eigen_impl::RotationQuaternion<PrimType_, Usage_>& q) {
    q.toImplementation().normalize();
  }
};

} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_EIGEN_ROTATIONQUATERNION_HPP_ */

