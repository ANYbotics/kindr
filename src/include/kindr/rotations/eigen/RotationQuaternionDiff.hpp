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
#ifndef KINDR_ROTATIONS_EIGEN_ROTATIONQUATERNIONDIFF_HPP_
#define KINDR_ROTATIONS_EIGEN_ROTATIONQUATERNIONDIFF_HPP_

#include <Eigen/Core>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationDiffBase.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include "kindr/quaternions/QuaternionEigen.hpp"
#include "kindr/linear_algebra/LinearAlgebra.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {

/*! \class RotationQuaternionDiff
 * \brief Time derivative of a rotation quaternion.
 *
 * This class implements the time derivative of a rotation quaterion using a quaternions::eigen_impl::Quaternion<Scalar> as data storage.
 *
 * \tparam PrimType_  Primitive data type of the coordinates.
 * \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class RotationQuaternionDiff : public RotationQuaternionDiffBase<RotationQuaternionDiff<PrimType_, Usage_>,Usage_>, private quaternions::eigen_impl::Quaternion<PrimType_> {
 private:
  /*! \brief The base type.
   */
  typedef quaternions::eigen_impl::Quaternion<PrimType_> Base;

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
  typedef Eigen::Matrix<PrimType_, 3, 1> Imaginary;

  //! quaternion as 4x1 matrix: [w; x; y; z]
  typedef Eigen::Matrix<PrimType_,4,1> Vector4;

  /*! \brief Default constructor sets all derivatives to zero
   *
   */
  RotationQuaternionDiff()
    : Base() {
  }

  explicit RotationQuaternionDiff(const Base& other) // explicit on purpose
    : Base(other) {
  }

  /*! \brief Constructor using four scalars.
   *  \param w     first entry of the derivative of the quaternion
   *  \param x     second entry of the derivative of the quaternion
   *  \param y     third entry of the derivative of the quaternion
   *  \param z     fourth entry of the derivative of the quaternion
   */
  RotationQuaternionDiff(Scalar w, Scalar x, Scalar y, Scalar z)
    : Base(w,x,y,z) {
  }

  RotationQuaternionDiff(const Vector4 & vec)
    : Base(vec(0),vec(1),vec(2),vec(3)) {
  }

  RotationQuaternionDiff(const Scalar& real, const Imaginary& imag)
    : Base(real, imag)  {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit RotationQuaternionDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RotationDiffBase<OtherDerived_, Usage_>& other)
    : Base(internal::RotationDiffConversionTraits<RotationQuaternionDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toQuaternion()){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RotationDiffConversionTraits<OtherDerived_, RotationQuaternionDiff, RotationDerived_>::convert(rotation.derived(), *this);
  }


  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return this->toQuaternion().toImplementation();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return this->toQuaternion().toImplementation();
  }

  /*! \brief Cast to the base type.
   *  \returns the quaternion that contains the derivative (recommended only for advanced users)
   */
  Base& toQuaternion()  {
    return static_cast<Base&>(*this);
  }


  /*! \brief Cast to the base type.
   *  \returns the quaternion that contains the derivative (recommended only for advanced users)
   */
  const Base& toQuaternion() const {
    return static_cast<const Base&>(*this);
  }


  using Base::w;
  using Base::x;
  using Base::y;
  using Base::z;

  using Base::real;
  using Base::imaginary;
  using Base::vector;

  /*! \brief Sets all time derivatives to zero.
   *  \returns reference
   */
  RotationQuaternionDiff& setZero() {
    Base::setZero();
    return *this;
  }


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationQuaternionDiff& diff) {
    out << diff.toQuaternion();
    return out;
  }
};


//! \brief Time derivative of a rotation quaternion with primitive type double
typedef RotationQuaternionDiff<double, RotationUsage::PASSIVE> RotationQuaternionDiffPD;
//! \brief Time derivative of a rotation quaternion with primitive type float
typedef RotationQuaternionDiff<float, RotationUsage::PASSIVE> RotationQuaternionDiffPF;
//! \brief Time derivative of a rotation quaternion with primitive type double
typedef RotationQuaternionDiff<double, RotationUsage::ACTIVE> RotationQuaternionDiffAD;
//! \brief Time derivative of a rotation quaternion with primitive type float
typedef RotationQuaternionDiff<float, RotationUsage::ACTIVE> RotationQuaternionDiffAF;

} // namespace eigen_impl

namespace internal {


template<typename PrimType_, enum RotationUsage Usage_>
class RotationDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>, eigen_impl::LocalAngularVelocity<PrimType_, Usage_>, eigen_impl::RotationQuaternion<PrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternionDiff<PrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<PrimType_, Usage_>& rquat, const eigen_impl::LocalAngularVelocity<PrimType_, Usage_>& angularVelocity) {
    return eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>(quaternions::eigen_impl::Quaternion<PrimType_>(0.5*rquat.getLocalQuaternionDiffMatrix().transpose()*angularVelocity.toImplementation()));
  }
};


//template<typename PrimType_, enum RotationUsage Usage_>
//class RotationDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>, eigen_impl::RotationVectorDiff<PrimType_, Usage_>, eigen_impl::RotationVector<PrimType_, Usage_>> {
// public:
//  inline static eigen_impl::RotationQuaternionDiff<PrimType_, Usage_> convert(const eigen_impl::RotationVector<PrimType_, Usage_>& rotationVector, const eigen_impl::RotationVectorDiff<PrimType_, Usage_>& rotationVectorDiff) {
//    typedef typename eigen_impl::RotationVector<PrimType_, Usage_>::Scalar Scalar;
//    const PrimType_ v = rotationVector.vector().norm();
//    const PrimType_ v1 = rotationVector.x();
//    const PrimType_ v2 = rotationVector.y();
//    const PrimType_ v3 = rotationVector.z();
//    const PrimType_ dv1 = rotationVectorDiff.x();
//    const PrimType_ dv2 = rotationVectorDiff.y();
//    const PrimType_ dv3 = rotationVectorDiff.z();
//
//    if (v < common::internal::NumTraits<Scalar>::dummy_precision()) {
//      Eigen::Matrix<PrimType_, 3, 3> G_v0;
//      G_v0 = -v1, -v2, -v3,
//              2.0, 2.0, 0.0,
//              0.0, 0.0, 2.0;
//      G_v0 *=0.25;
//      return eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>(G_v0*rotationVectorDiff.toImplementation());
//    }
//
//
//
//    const PrimType_ t2 = 1.0/v;
//    const PrimType_ t3 = v*(1.0/2.0);
//    const PrimType_ t4 = sin(t3);
//    const PrimType_ t5 = 1.0/(v*v*v);
//    const PrimType_ t6 = t4*2.0;
//    const PrimType_ t7 = cos(t3);
//    const PrimType_ t9 = t7*v;
//    const PrimType_ t8 = t6-t9;
//    const PrimType_ t10 = t2*t4;
//    const PrimType_ w = dv1*t2*t4*v1*(-1.0/2.0)-dv2*t2*t4*v2*(1.0/2.0)-dv3*t2*t4*v3*(1.0/2.0);
//    const PrimType_ x = dv1*(t10-t5*t8*(v1*v1)*(1.0/2.0))-dv2*t5*t8*v1*v2*(1.0/2.0)-dv3*t5*t8*v1*v3*(1.0/2.0);
//    const PrimType_ y = dv2*(t10-t5*t8*(v2*v2)*(1.0/2.0))-dv1*t5*t8*v1*v2*(1.0/2.0)-dv3*t5*t8*v2*v3*(1.0/2.0);
//    const PrimType_ z = dv3*(t10-t5*t8*(v3*v3)*(1.0/2.0))-dv1*t5*t8*v1*v3*(1.0/2.0)-dv2*t5*t8*v2*v3*(1.0/2.0);
//
//
//    return eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>(w, x, y, z);
//  }
//};

//template<typename PrimType_, enum RotationUsage Usage_>
//class RotationDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>, eigen_impl::RotationVectorDiff<PrimType_, Usage_>, eigen_impl::RotationQuaternion<PrimType_, Usage_>> {
// public:
//  inline static eigen_impl::RotationQuaternionDiff<PrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<PrimType_, Usage_>& rquat, const eigen_impl::RotationVectorDiff<PrimType_, Usage_>& rotationVectorDiff) {
//    return RotationDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>, eigen_impl::RotationVectorDiff<PrimType_, Usage_>, eigen_impl::RotationVector<PrimType_, Usage_>>::convert(eigen_impl::RotationVector<PrimType_, Usage_>(rquat), rotationVectorDiff);
//  }
//};



} // namespace internal
} // namespace rotations
} // namespace kindr

#endif /* KINDR_ROTATIONS_EIGEN_ROTATIONQUATERNIONDIFF_HPP_ */
