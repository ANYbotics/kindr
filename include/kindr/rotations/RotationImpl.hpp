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

#include "kindr/common/common.hpp"
#include "kindr/math/LinearAlgebra.hpp"
#include "kindr/common/assert_macros.hpp"
#include "kindr/rotations/RotationBase.hpp"
#include "kindr/vectors/VectorBase.hpp"

namespace kindr {

template<typename PrimType_>
class AngleAxis;

template<typename PrimType_>
class RotationVector;

template<typename PrimType_>
class RotationQuaternion;

template<typename PrimType_>
class RotationMatrix;

template<typename PrimType_>
class EulerAnglesZyx;

template<typename PrimType_>
class EulerAnglesXyz;


namespace internal {

template<typename PrimType_>
class get_scalar<Eigen::Matrix<PrimType_, 3, 1>> {
 public:
  typedef PrimType_ Scalar;
};


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Multiplication Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/*! \brief Multiplication of two rotations with different parameterizations
 */
template<typename Left_, typename Right_>
class MultiplicationTraits<RotationBase<Left_>, RotationBase<Right_> > {
 public:
  //! Default multiplication of rotations converts the representations of the rotations to rotation quaternions and multiplies them
  inline static Left_ mult(const RotationBase<Left_>& lhs, const RotationBase<Right_>& rhs) {
      return Left_(RotationQuaternion<typename Left_::Scalar>(
                  (RotationQuaternion<typename Left_::Scalar>(lhs.derived())).toImplementation() *
                  (RotationQuaternion<typename Right_::Scalar>(rhs.derived())).toImplementation()
                  ));

  }
};

/*! \brief Multiplication of two rotations with the same parameterization
 */
template<typename LeftAndRight_>
class MultiplicationTraits<RotationBase<LeftAndRight_>, RotationBase<LeftAndRight_> > {
 public:
  //! Default multiplication of rotations converts the representations of the rotations to rotation quaternions and multiplies them
  inline static LeftAndRight_ mult(const RotationBase<LeftAndRight_>& lhs, const RotationBase<LeftAndRight_>& rhs) {
      return LeftAndRight_(RotationQuaternion<typename LeftAndRight_::Scalar>(
                          (RotationQuaternion<typename LeftAndRight_::Scalar>(lhs.derived())).toImplementation() *
                          (RotationQuaternion<typename LeftAndRight_::Scalar>(rhs.derived())).toImplementation()
                          ));

  }
};


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename Rotation_>
class RotationTraits<RotationBase<Rotation_>> {
 public:
  //! Default rotation operator converts the representation of the rotation to a rotation matrix and multiplies it with the matrix
  template<typename get_matrix3X<Rotation_>::IndexType Cols>
  inline static typename get_matrix3X<Rotation_>::template Matrix3X<Cols> rotate(const RotationBase<Rotation_>& rotation, const typename internal::get_matrix3X<Rotation_>::template Matrix3X<Cols>& m){
      return RotationMatrix<typename Rotation_::Scalar>(rotation.derived()).toImplementation()*m;
  }
  //! Default rotation operator converts the representation of the rotation to a rotation matrix and multiplies it with the matrix
  template<typename Vector_>
  inline static Vector_ rotate(const RotationBase<Rotation_>& rotation, const Vector_& vector){
    return static_cast<Vector_>(rotation.derived().rotate(vector.toImplementation()));
  }

};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Comparison Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/*! \brief Compares two rotations.
 *
 */
template<typename Left_, typename Right_>
class ComparisonTraits<RotationBase<Left_>, RotationBase<Right_>> {
 public:
  inline static bool isEqual(const RotationBase<Left_>& left, const RotationBase<Right_>& right) {
    return left.derived().toImplementation() == static_cast<Left_>(right).toImplementation();
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Map Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename Rotation_>
class MapTraits<RotationBase<Rotation_>> {
 public:
  inline static Rotation_ get_exponential_map(const typename internal::get_matrix3X<Rotation_>::template Matrix3X<1>& vector) {
    typedef typename get_scalar<Rotation_>::Scalar Scalar;
    return Rotation_(RotationVector<Scalar>(vector));
  }

  inline static typename internal::get_matrix3X<Rotation_>::template Matrix3X<1> get_logarithmic_map(const Rotation_& rotation) {
    typedef typename get_scalar<Rotation_>::Scalar Scalar;
    return RotationVector<Scalar>(rotation).getUnique().toImplementation(); // unique is required
  }

};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Box Operation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename Left_, typename Right_>
class BoxOperationTraits<RotationBase<Left_>, RotationBase<Right_>> {
 public:
  inline static typename internal::get_matrix3X<Left_>::template Matrix3X<1> box_minus(const RotationBase<Left_>& lhs, const RotationBase<Right_>& rhs) {
    return (lhs.derived()*rhs.derived().inverted()).logarithmicMap();
  }

  inline static  Left_ box_plus(const RotationBase<Left_>& rotation, const typename internal::get_matrix3X<Left_>::template Matrix3X<1>& vector) {
    return Left_(MapTraits<RotationBase<Left_>>::get_exponential_map(vector)*rotation.derived());
  }
};



/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * SetFromVectors Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename Rotation_>
class SetFromVectorsTraits<RotationBase<Rotation_>> {
 public:
  template<typename PrimType_>
  inline static void setFromVectors(Rotation_& rot, const Eigen::Matrix<PrimType_, 3, 1>& v1, const Eigen::Matrix<PrimType_, 3, 1>& v2) {
    KINDR_ASSERT_TRUE(std::runtime_error,  v1.norm()*v2.norm() != static_cast<PrimType_>(0.0), "At least one vector has zero length.");

    Eigen::Quaternion<PrimType_> eigenQuat;
    eigenQuat.setFromTwoVectors(v1, v2);
    rot = kindr::RotationQuaternion<PrimType_>(eigenQuat);
//
//    const PrimType_ angle = acos(v1.dot(v2)/temp);
//    const PrimType_ tol = 1e-3;
//
//    if(0 <= angle && angle < tol) {
//      rot.setIdentity();
//    } else if(M_PI - tol < angle && angle < M_PI + tol) {
//      rot = AngleAxis<PrimType_>(angle, 1, 0, 0);
//    } else {
//      const Eigen::Matrix<PrimType_, 3, 1> axis = (v1.cross(v2)).normalized();
//      rot = AngleAxis<PrimType_>(angle, axis);
//    }
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Disparity Angle Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/*! \brief Compute the disparity angle between two rotations.
 *
 */
template<typename Left_, typename Right_>
class DisparityAngleTraits<RotationBase<Left_>, RotationBase<Right_>> {
 public:
  /*! \brief Gets the disparity angle between two rotations.
   *
   *  The disparity angle is defined as the angle of the angle-axis representation of the concatenation of
   *  the first rotation and the inverse of the second rotation. If the disparity angle is zero,
   *  the rotations are equal.
   *  \returns disparity angle in [-pi,pi) @todo: is this range correct?
   */
  inline static typename Left_::Scalar compute(const RotationBase<Left_>& left, const RotationBase<Right_>& right) {
    typedef typename Left_::Scalar Scalar;
    return std::abs(floatingPointModulo(AngleAxis<Scalar>(left.derived()*right.derived().inverted()).angle() + Scalar(M_PI), Scalar(2.0*M_PI))-M_PI);
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Random Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename Rotation_>
class RandomTraits<RotationBase<Rotation_>> {
 public:
  inline static void set_random(Rotation_& rot) {
    typedef typename Rotation_::Scalar Scalar;
    Eigen::Matrix<Scalar, 3, 1> vector;
    setUniformRandom<Scalar, 3>(vector, -M_PI, M_PI);
    rot = Rotation_(RotationVector<Scalar>(vector));
  }
};

} // namespace internal
} // namespace kindr
