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
#ifndef KINDR_ROTATIONS_EIGEN_ROTATIONVECTORDIFF_HPP_
#define KINDR_ROTATIONS_EIGEN_ROTATIONVECTORDIFF_HPP_

#include <Eigen/Core>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RDiffBase.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include "kindr/linear_algebra/LinearAlgebra.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {


template<typename PrimType, enum RotationUsage Usage>
class RotationQuaternionDiff;

template<typename PrimType, enum RotationUsage Usage>
class RotationMatrixDiff;

template<typename PrimType, enum RotationUsage Usage>
class AngleAxisDiff;

template<typename PrimType, enum RotationUsage Usage>
class RotationVectorDiff;

template<typename PrimType, enum RotationUsage Usage>
class EulerAnglesZyxDiff;

template<typename PrimType, enum RotationUsage Usage>
class EulerAnglesXyzDiff;


template<typename PrimType_, enum RotationUsage Usage_>
class RotationVectorDiff : public RotationVectorDiffBase<RotationVectorDiff<PrimType_, Usage_>,Usage_> {
 private:
  /*! \brief The base type.
   */
  typedef typename Eigen::Matrix<PrimType_, 3, 1> Base;

 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;

  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief data container
   */
  Base vector_;

  RotationVectorDiff()
    : vector_(Base::Zero()) {
  }

  explicit RotationVectorDiff(const Base& other) // explicit on purpose
    : vector_(other) {
  }

  /*! \brief Constructor using four scalars.
   *  \param v1      first entry of the time derivative of the rotation axis vector
   *  \param v2      second entry of the time derivative of the rotation axis vector
   *  \param v3      third entry of the time derivative of the rotation axis vector
   */
  RotationVectorDiff(Scalar v1, Scalar v2, Scalar v3)
    : vector_(v1,v2,v3) {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit RotationVectorDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : vector_(internal::RDiffConversionTraits<RotationVectorDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toImplementation()){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RDiffConversionTraits<OtherDerived_, RotationVectorDiff, RotationDerived_>::convert(rotation.derived(), *this);
  }


  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(vector_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return vector_;
  }


  inline Scalar x() const {
    return toImplementation()(0);
  }


  inline Scalar y() const {
    return toImplementation()(1);
  }


  inline Scalar z() const {
    return toImplementation()(2);
  }


  /*! \brief Sets all time derivatives to zero.
   *  \returns reference
   */
  RotationVectorDiff& setZero() {
    vector_.setZero();
    return *this;
  }


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationVectorDiff& diff) {
    out << diff.toImplementation().transpose();
    return out;
  }
};


//! \brief Time derivative of a rotation vector with primitive type double
typedef RotationVectorDiff<double, RotationUsage::PASSIVE> RotationVectorDiffPD;
//! \brief Time derivative of a rotation vector with primitive type float
typedef RotationVectorDiff<float, RotationUsage::PASSIVE> RotationVectorDiffPF;
//! \brief Time derivative of a rotation vector with primitive type double
typedef RotationVectorDiff<double, RotationUsage::ACTIVE> RotationVectorDiffAD;
//! \brief Time derivative of a rotation vector with primitive type float
typedef RotationVectorDiff<float, RotationUsage::ACTIVE> RotationVectorDiffAF;


} // namespace eigen_impl

namespace internal {



template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>& rotationVector, const eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>& angularVelocity) {
    typedef typename eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>::Implementation Vector;
    typedef typename Eigen::Matrix<PrimType_, 3, 3> Matrix3x3;

    // not tested:
//    const Vector rv = rotationVector.toImplementation();
//    const PrimType_ angle = rv.norm();
//
//    if (angle < 1e-14) {
//      return eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>(angularVelocity.toImplementation());
//    }
//
//    const Matrix3x3 rv_hat = linear_algebra::getSkewMatrixFromVector(rv);
//    const Vector rvDiff = (Matrix3x3::Identity() + 0.5*rv_hat + (1.0/(angle*angle) - sin(angle)/(2.0*angle*(1.0-cos(angle))))*rv_hat*rv_hat)*angularVelocity.toImplementation();
//    return eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>(rvDiff);

    const PrimType_ v = rotationVector.vector().norm();
    if (v < 1e-14) {
      return eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>(angularVelocity.toImplementation());
    }
    const PrimType_ w1 = angularVelocity.x();
    const PrimType_ w2 = angularVelocity.y();
    const PrimType_ w3 = angularVelocity.z();

    const PrimType_ v1 = rotationVector.firstEntry();
    const PrimType_ v2 = rotationVector.secondEntry();
    const PrimType_ v3 = rotationVector.thirdEntry();
    const PrimType_ t6 = v*(1.0/2.0);
    const PrimType_ t2 = sin(t6);
    const PrimType_ t3 = t2*t2;
    const PrimType_ t4 = v2*v2;
    const PrimType_ t5 = v*v;
    const PrimType_ t7 = cos(t6);
    const PrimType_ t8 = v1*v1;
    const PrimType_ t9 = v3*v3;
    const PrimType_ t10 = t7*t7;
    const PrimType_ t11 = t3*t3;
    const PrimType_ t12 = t8*t8;
    const PrimType_ t13 = t4*t4;
    const PrimType_ t14 = t9*t9;
    const PrimType_ t15 = t5*t5;
    const PrimType_ t16 = t10*t10;
    const PrimType_ t17 = t2*t11*t12*v;
    const PrimType_ t18 = t2*t11*t13*v;
    const PrimType_ t19 = t2*t11*t14*v;
    const PrimType_ t20 = t3*t7*t10*t15*2.0;
    const PrimType_ t21 = t2*t4*t8*t11*v*2.0;
    const PrimType_ t22 = t2*t8*t9*t11*v*2.0;
    const PrimType_ t23 = t2*t4*t9*t11*v*2.0;
    const PrimType_ t24 = t2*t3*t5*t8*t10*v;
    const PrimType_ t25 = t2*t3*t4*t5*t10*v;
    const PrimType_ t26 = t2*t3*t5*t9*t10*v;
    const PrimType_ t27 = t5*t7*t8*t11*2.0;
    const PrimType_ t28 = t2*t3*t10*t12*v;
    const PrimType_ t29 = t4*t5*t7*t11*2.0;
    const PrimType_ t30 = t2*t5*t8*t16*v;
    const PrimType_ t31 = t2*t3*t10*t13*v;
    const PrimType_ t32 = t5*t7*t9*t11*2.0;
    const PrimType_ t33 = t2*t4*t5*t16*v;
    const PrimType_ t34 = t2*t3*t10*t14*v;
    const PrimType_ t35 = t2*t5*t9*t16*v;
    const PrimType_ t36 = t2*t3*t4*t8*t10*v*2.0;
    const PrimType_ t37 = t2*t3*t8*t9*t10*v*2.0;
    const PrimType_ t38 = t2*t3*t4*t9*t10*v*2.0;
    const PrimType_ t41 = t7*t11*t12*2.0;
    const PrimType_ t42 = t7*t11*t13*2.0;
    const PrimType_ t43 = t7*t11*t14*2.0;
    const PrimType_ t44 = t3*t5*t7*t8*t10*2.0;
    const PrimType_ t45 = t3*t4*t5*t7*t10*2.0;
    const PrimType_ t46 = t3*t5*t7*t9*t10*2.0;
    const PrimType_ t47 = t4*t7*t8*t11*4.0;
    const PrimType_ t48 = t7*t8*t9*t11*4.0;
    const PrimType_ t49 = t4*t7*t9*t11*4.0;
    const PrimType_ t39 = t17+t18+t19+t20+t21+t22+t23+t24+t25+t26+t27+t28+t29+t30+t31+t32+t33+t34+t35+t36+t37+t38-t41-t42-t43-t44-t45-t46-t47-t48-t49;
    const PrimType_ t40 = 1.0/t39;
    const PrimType_ t50 = t9*v*v3*(1.0/2.0);
    const PrimType_ t51 = t8*v*v3*(1.0/2.0);
    const PrimType_ t52 = t4*v*v3*(1.0/2.0);
    const PrimType_ t53 = t9*v*v3;
    const PrimType_ t54 = t8*v*v3;
    const PrimType_ t55 = t4*v*v3;
    const PrimType_ t56 = t8*v3;
    const PrimType_ t57 = t4*v3;
    const PrimType_ t58 = t9*v3;
    const PrimType_ t59 = t7*t10*t15*v1*v2*(1.0/2.0);
    const PrimType_ t60 = t9*v*(1.0/2.0);
    const PrimType_ t61 = t9*v;
    const PrimType_ t62 = t8*v*v1*(1.0/2.0);
    const PrimType_ t63 = t4*v*v1*(1.0/2.0);
    const PrimType_ t64 = t9*v*v1*(1.0/2.0);
    const PrimType_ t65 = t8*v*v1;
    const PrimType_ t66 = t4*v*v1;
    const PrimType_ t67 = t9*v*v1;
    const PrimType_ t68 = t4*v1;
    const PrimType_ t69 = t9*v1;
    const PrimType_ t70 = t8*v1;
    const PrimType_ t71 = t7*t10*t15*v2*v3*(1.0/2.0);
    const PrimType_ t72 = t4*v*v2*(1.0/2.0);
    const PrimType_ t73 = t8*v*v2*(1.0/2.0);
    const PrimType_ t74 = t9*v*v2*(1.0/2.0);
    const PrimType_ t75 = v*v1*v3;
    const PrimType_ t76 = t4*v*v2;
    const PrimType_ t77 = t8*v*v2;
    const PrimType_ t78 = t9*v*v2;
    const PrimType_ t79 = v*v1*v3*2.0;
    const PrimType_ t80 = t8*v2;
    const PrimType_ t81 = t9*v2;
    const PrimType_ t82 = t5*v1*v3*(1.0/2.0);
    const PrimType_ t83 = t4*v2;
    const PrimType_ t84 = t7*t10*t15*v1*v3*(1.0/2.0);
    const PrimType_ t85 = t8*v*(1.0/2.0);
    const PrimType_ t86 = t4*v*(1.0/2.0);
    const PrimType_ t87 = t8*v;
    const PrimType_ t88 = t4*v;
    const PrimType_ dv1 = -t40*w2*(t59+t2*t3*t5*(t50+t51+t52-v*v1*v2)+t2*t5*t10*(t53+t54+t55-v*v1*v2*2.0)*(1.0/2.0)-t3*t5*t7*(t56+t57+t58-t5*v3-t5*v1*v2*(1.0/2.0)))-t40*w3*(t84+t3*t5*t7*(t80+t81+t82+t83-t5*v2)-t2*t3*t5*(t72+t73+t74+t75)-t2*t5*t10*(t76+t77+t78+t79)*(1.0/2.0))+t40*w1*(-t2*t5*t10*v*(t4-t5+t9)+t3*t5*t7*v*(t61+t88)*(1.0/2.0)+t5*t7*t10*v*(t60+t86)+t2*t3*t5*t8*v);
    const PrimType_ dv2 = -t40*w1*(t59-t2*t3*t5*(t50+t51+t52+v*v1*v2)-t2*t5*t10*(t53+t54+t55+v*v1*v2*2.0)*(1.0/2.0)+t3*t5*t7*(t56+t57+t58-t5*v3+t5*v1*v2*(1.0/2.0)))-t40*w3*(t71+t2*t3*t5*(t62+t63+t64-v*v2*v3)+t2*t5*t10*(t65+t66+t67-v*v2*v3*2.0)*(1.0/2.0)-t3*t5*t7*(t68+t69+t70-t5*v1-t5*v2*v3*(1.0/2.0)))+t40*w2*(-t2*t5*t10*v*(-t5+t8+t9)+t3*t5*t7*v*(t61+t87)*(1.0/2.0)+t5*t7*t10*v*(t60+t85)+t2*t3*t4*t5*v);
    const PrimType_ dv3 = -t40*w2*(t71-t2*t3*t5*(t62+t63+t64+v*v2*v3)-t2*t5*t10*(t65+t66+t67+v*v2*v3*2.0)*(1.0/2.0)+t3*t5*t7*(t68+t69+t70-t5*v1+t5*v2*v3*(1.0/2.0)))-t40*w1*(t84+t2*t3*t5*(t72+t73+t74-t75)+t2*t5*t10*(t76+t77+t78-t79)*(1.0/2.0)-t3*t5*t7*(t80+t81-t82+t83-t5*v2))+t40*w3*(-t2*t5*t10*v*(t4-t5+t8)+t3*t5*t7*v*(t87+t88)*(1.0/2.0)+t5*t7*t10*v*(t85+t86)+t2*t3*t5*t9*v);
    return eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>(dv1, dv2, dv3);

  }
};



} // namespace internal
} // namespace rotations
} // namespace kindr




#endif /* KINDR_ROTATIONS_EIGEN_ROTATIONVECTORDIFF_HPP_ */
