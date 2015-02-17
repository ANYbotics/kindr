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

#ifndef KINDR_ROTATIONS_ROTATIONEIGEN_HPP_
#define KINDR_ROTATIONS_ROTATIONEIGEN_HPP_

#include "kindr/common/assert_macros.hpp"
#include "kindr/rotations/RotationBase.hpp"
//#include "kindr/positions/PositionEigen.hpp"
#include "kindr/linear_algebra/LinearAlgebra.hpp"

#include "kindr/vectors/VectorBase.hpp"

namespace kindr {
namespace rotations {
//! Implementation of rotations based on the C++ Eigen library
namespace eigen_impl {

template<typename PrimType_, enum RotationUsage Usage_>
class AngleAxis;

template<typename PrimType_, enum RotationUsage Usage_>
class RotationVector;

template<typename PrimType_, enum RotationUsage Usage_>
class RotationQuaternion;

template<typename PrimType_, enum RotationUsage Usage_>
class RotationMatrix;

template<typename PrimType_, enum RotationUsage Usage_>
class EulerAnglesZyx;

template<typename PrimType_, enum RotationUsage Usage_>
class EulerAnglesXyz;


} // namespace eigen_impl




namespace internal {

//template<typename PrimType_>
//class get_position3<positions::eigen_impl::Position3<PrimType_>>{
// private:
//  typedef typename positions::eigen_impl::Position3<PrimType_> Position;
//  typedef typename Position::Implementation Matrix3X;
// public:
//  static const Matrix3X& get_matrix3(const Position& position) {
//    return position.toImplementation();
//  }
//};

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
template<typename Left_, typename Right_, enum RotationUsage Usage_>
class MultiplicationTraits<RotationBase<Left_, Usage_>, RotationBase<Right_, Usage_>> {
 public:
  //! Default multiplication of rotations converts the representations of the rotations to rotation quaternions and multiplies them
  inline static Left_ mult(const RotationBase<Left_, Usage_>& lhs, const RotationBase<Right_, Usage_>& rhs) {
    if(Usage_ == RotationUsage::ACTIVE) {
      return Left_(typename eigen_impl::RotationQuaternion<typename Left_::Scalar,  Usage_>(
                  (typename eigen_impl::RotationQuaternion<typename Left_::Scalar,  Usage_>(lhs.derived())).toImplementation() *
                  (typename eigen_impl::RotationQuaternion<typename Right_::Scalar, Usage_>(rhs.derived())).toImplementation()
                  ));
    } else {
      return Left_(typename eigen_impl::RotationQuaternion<typename Left_::Scalar,  Usage_>(
                  (typename eigen_impl::RotationQuaternion<typename Right_::Scalar, Usage_>(rhs.derived())).toImplementation() *
                  (typename eigen_impl::RotationQuaternion<typename Left_::Scalar,  Usage_>(lhs.derived())).toImplementation()
                  ));
    }
  }
};

/*! \brief Multiplication of two rotations with the same parameterization
 */
template<typename LeftAndRight_, enum RotationUsage Usage_>
class MultiplicationTraits<RotationBase<LeftAndRight_, Usage_>, RotationBase<LeftAndRight_, Usage_>> {
 public:
  inline static LeftAndRight_ mult(const RotationBase<LeftAndRight_, Usage_>& lhs, const RotationBase<LeftAndRight_, Usage_>& rhs) {
    if(Usage_ == RotationUsage::ACTIVE) {
      return LeftAndRight_(typename eigen_impl::RotationQuaternion<typename LeftAndRight_::Scalar, Usage_>(
                          (typename eigen_impl::RotationQuaternion<typename LeftAndRight_::Scalar, Usage_>(lhs.derived())).toImplementation() *
                          (typename eigen_impl::RotationQuaternion<typename LeftAndRight_::Scalar, Usage_>(rhs.derived())).toImplementation()
                          ));
    } else {
      return LeftAndRight_(typename eigen_impl::RotationQuaternion<typename LeftAndRight_::Scalar, Usage_>(
                          (typename eigen_impl::RotationQuaternion<typename LeftAndRight_::Scalar, Usage_>(rhs.derived())).toImplementation() *
                          (typename eigen_impl::RotationQuaternion<typename LeftAndRight_::Scalar, Usage_>(lhs.derived())).toImplementation()
                          ));
    }
  }
};


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Comparison Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/*! \brief Compares two rotations.
 *
 */
template<typename Left_, typename Right_, enum RotationUsage Usage_>
class ComparisonTraits<RotationBase<Left_, Usage_>, RotationBase<Right_, Usage_>> {
 public:
  /*! \brief Gets the disparity angle between two rotations.
   *
   *  The disparity angle is defined as the angle of the angle-axis representation of the concatenation of
   *  the first rotation and the inverse of the second rotation. If the disparity angle is zero,
   *  the rotations are equal.
   *  \returns disparity angle in [-pi,pi) @todo: is this range correct?
   */
  inline static typename Left_::Scalar get_disparity_angle(const RotationBase<Left_, Usage_>& left, const RotationBase<Right_, Usage_>& right) {
    return fabs(common::floatingPointModulo(eigen_impl::AngleAxis<typename Left_::Scalar,  Usage_>(left.derived()*right.derived().inverted()).angle() + M_PI,2*M_PI)-M_PI);
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Map Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename Rotation_>
class MapTraits<RotationBase<Rotation_, Rotation_::Usage>> {
 public:

  inline static Rotation_ set_exponential_map(const typename internal::get_matrix3X<Rotation_>::template Matrix3X<1>& vector) {
    typedef typename get_scalar<Rotation_>::Scalar Scalar;
//    if (Rotation_::Usage == RotationUsage::ACTIVE) {
//      return Rotation_(eigen_impl::RotationVector<Scalar, Rotation_::Usage>(vector));
//    }
//    if (Rotation_::Usage == RotationUsage::PASSIVE) {
//      return Rotation_(eigen_impl::RotationVector<Scalar, Rotation_::Usage>(vector));
//    }
    return Rotation_(eigen_impl::RotationVector<Scalar, Rotation_::Usage>(vector));
  }

//  inline static typename internal::get_matrix3X<Rotation_>::template Matrix3X<1> get_logarithmic_map(const Rotation_& rotation) {
//    typedef typename get_scalar<Rotation_>::Scalar Scalar;
//    eigen_impl::RotationVector<Scalar, Rotation_::Usage> rotationVector(rotation);
//    return rotationVector.getUnique().toImplementation();
//  }

  inline static typename internal::get_matrix3X<Rotation_>::template Matrix3X<1> get_logarithmic_map(const Rotation_& rotation) {
    typedef typename get_scalar<Rotation_>::Scalar Scalar;
// ok
    return eigen_impl::RotationVector<Scalar, Rotation_::Usage>(rotation).getUnique().toImplementation(); // unique?
//    return eigen_impl::RotationVector<Scalar, Rotation_::Usage>(rotation).toImplementation();


  }

};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Box Operation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename Left_, typename Right_, enum RotationUsage Usage_>
class BoxOperationTraits<RotationBase<Left_, Usage_>, RotationBase<Right_, Usage_>> {
 public:
  inline static typename internal::get_matrix3X<Left_>::template Matrix3X<1> box_minus(const RotationBase<Left_, Usage_>& lhs, const RotationBase<Right_, Usage_>& rhs) {
    return (lhs.derived()*rhs.derived().inverted()).logarithmicMap();
  }

  inline static  Left_ box_plus(const RotationBase<Left_, Usage_>& rotation, const typename internal::get_matrix3X<Left_>::template Matrix3X<1>& vector) {
    return Left_(MapTraits<RotationBase<Left_,Usage_>>::set_exponential_map(vector)*rotation.derived());
  }
};


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename Rotation_, enum RotationUsage Usage_>
class RotationTraits<RotationBase<Rotation_, Usage_>> {
 public:
  template<typename get_matrix3X<Rotation_>::IndexType Cols>
  inline static typename get_matrix3X<Rotation_>::template Matrix3X<Cols> rotate(const RotationBase<Rotation_, Usage_>& rotation, const typename internal::get_matrix3X<Rotation_>::template Matrix3X<Cols>& m){
    if(Usage_ == RotationUsage::ACTIVE){
      return eigen_impl::RotationMatrix<typename Rotation_::Scalar, Usage_>(rotation.derived()).toImplementation()*m;
    } else {
      return eigen_impl::RotationMatrix<typename Rotation_::Scalar, Usage_>(rotation.derived().inverted()).toImplementation()*m;
    }
  }

//  template<typename Vector_>
//  inline static Vector_ rotate(const RotationBase<Rotation_, Usage_>& rotation, const Vector_& vector){
//    if(Usage_ == RotationUsage::ACTIVE){
//      return Vector_(rotation.derived().toImplementation()*vector.toImplementation());
//    } else {
//      return Vector_(rotation.derived().inverted().toImplementation()*vector.toImplementation());
//    }
//  }


  template<typename Vector_>
  inline static Vector_ rotate(const RotationBase<Rotation_, Usage_>& rotation, const Vector_& vector){
    return static_cast<Vector_>(rotation.derived().rotate(vector.toImplementation()));
//    if(Usage_ == RotationUsage::ACTIVE){
//
//    } else {
//      return static_cast<Vector_>(eigen_impl::RotationMatrix<typename Rotation_::Scalar, Usage_>(rotation.derived().inverted()).toImplementation()*vector.toImplementation());
//    }
  }

};



/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * SetFromVectors Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename Rotation_, enum RotationUsage Usage_>
class SetFromVectorsTraits<RotationBase<Rotation_, Usage_>> {
 public:
  template<typename PrimType_>
  inline static void setFromVectors(Rotation_& rot, const Eigen::Matrix<PrimType_, 3, 1>& v1, const Eigen::Matrix<PrimType_, 3, 1>& v2) {
    KINDR_ASSERT_TRUE(std::runtime_error,  v1.norm()*v2.norm() != static_cast<PrimType_>(0.0), "At least one vector has zero length.");

    Eigen::Quaternion<PrimType_> eigenQuat;
    eigenQuat.setFromTwoVectors(v1, v2);
    rot = kindr::rotations::eigen_impl::RotationQuaternion<PrimType_, Usage_>(eigenQuat);
//
//    const PrimType_ angle = acos(v1.dot(v2)/temp);
//    const PrimType_ tol = 1e-3;
//
//    if(0 <= angle && angle < tol) {
//      rot.setIdentity();
//    } else if(M_PI - tol < angle && angle < M_PI + tol) {
//      rot = eigen_impl::AngleAxis<PrimType_, Usage_>(angle, 1, 0, 0);
//    } else {
//      const Eigen::Matrix<PrimType_, 3, 1> axis = (v1.cross(v2)).normalized();
//      rot = eigen_impl::AngleAxis<PrimType_, Usage_>(angle, axis);
//    }
  }
};



} // namespace internal



} // namespace rotations
} // namespace kindr


#include "kindr/rotations/eigen/AngleAxis.hpp"
#include "kindr/rotations/eigen/RotationVector.hpp"
#include "kindr/rotations/eigen/RotationQuaternion.hpp"
#include "kindr/rotations/eigen/RotationMatrix.hpp"
#include "kindr/rotations/eigen/EulerAnglesZyx.hpp"
#include "kindr/rotations/eigen/EulerAnglesXyz.hpp"




#endif /* KINDR_ROTATIONS_ROTATIONEIGEN_HPP_ */
