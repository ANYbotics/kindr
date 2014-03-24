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

#ifndef KINDR_ROTATIONS_EIGEN_ROTATIONVECTOR_HPP_
#define KINDR_ROTATIONS_EIGEN_ROTATIONVECTOR_HPP_

#include <cmath>

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationBase.hpp"
#include "kindr/rotations/eigen/RotationEigenFunctions.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {


/*! \class RotationVector
 * \brief Implementation of a rotation vector based on Eigen::Matrix<Scalar, 3, 1>
 *
 *  The rotation vector is a non-normalized three-dimensional vector.
 *  The direction of the vector specifies the rotation axis and the length the angle.
 *  This representation uses therefore three parameters.
 *  \see AngleAxis for an angle-axis representation with four parameters.
 *
 *  The following four typedefs are provided for convenience:
 *   - \ref eigen_impl::RotationVectorAD "RotationVectorAD" for active rotation and primitive type double
 *   - \ref eigen_impl::RotationVectorAF "RotationVectorAF" for active rotation and primitive type float
 *   - \ref eigen_impl::RotationVectorPD "RotationVectorPD" for passive rotation and primitive type double
 *   - \ref eigen_impl::RotationVectorPF "RotationVectorPF" for passive rotation and primitive type float
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class RotationVector : public RotationVectorBase<RotationVector<PrimType_, Usage_>, Usage_> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

  /*! Data container
   */
  Base vector_;

 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief Rotation Vector as 3x1-matrix
   */
  typedef Base Vector;

  /*! \brief Default constructor using identity rotation.
   */
  RotationVector()
    : vector_(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *
   *  \param x      first entry of the rotation vector
   *  \param y      second entry of the rotation vector
   *  \param z      third entry of the rotation vector
   */
  RotationVector(Scalar x, Scalar y, Scalar z)
    : vector_(x,y,z) {
  }


  /*! \brief Constructor using Eigen::Matrix<Scalar, 3, 1>.
   *
   *  \param other   Eigen::Matrix<Scalar, 3, 1>
   */
  explicit RotationVector(const Base& other) // explicit on purpose
    : vector_(other) {
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit RotationVector(const RotationBase<OtherDerived_, Usage_>& other)
    : vector_(internal::ConversionTraits<RotationVector, OtherDerived_>::convert(other.derived()).toImplementation()) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationVector& operator =(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toImplementation() = internal::ConversionTraits<RotationVector, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationVector& operator ()(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toImplementation() = internal::ConversionTraits<RotationVector, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  RotationVector inverted() const {
    return RotationVector(-this->toImplementation());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  RotationVector& invert() {
    *this = inverted();
    return *this;
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(vector_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return static_cast<const Implementation&>(vector_);
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  RotationVector& setIdentity() {
    vector_ = Base::Zero();
    return *this;
  }

  /*! \brief Returns the rotation vector.
   *  \returns the rotation vector (scalar)
   */
  inline const Vector vector() const {
    return this->toImplementation();
  }

  /*! \brief Sets the rotation vector.
   * \param x   first entry
   * \param y   second entry
   * \param z   third entry
   */
  inline void setVector(Scalar x, Scalar y, Scalar z) {
    vector_ << x, y, z;
  }

  /*! \brief Sets the rotation vector.
   */
  inline void setVector(const Implementation& vector) {
    vector_ = vector;
  }

  /*! \brief Returns the first entry of the rotation vector.
   *  \returns first entry of the rotation vector (scalar)
   */
  inline Scalar x() const {
    return vector_(0);
  }

  /*! \brief Returns the second entry of the rotation vector.
   *  \returns second entry of the rotation vector (scalar)
   */
  inline Scalar y() const {
    return vector_(1);
  }

  /*! \brief Returns the third entry of the rotation vector.
   *  \returns third entry of the rotation vector (scalar)
   */
  inline Scalar z() const {
    return vector_(2);
  }

  /*! \brief Sets the first entry of the rotation vector.
   */
  inline void setX(Scalar x) {
    vector_(0) = x;
  }

  /*! \brief Sets the second entry of the rotation vector.
   */
  inline void setY(Scalar y) {
    vector_(1) = y;
  }

  /*! \brief Sets the third entry of the rotation vector.
   */
  inline void setZ(Scalar z) {
    vector_(2) = z;
  }

  /*! \brief Returns a unique rotation vector with norm in [0,pi).
   *  This function is used to compare different rotations.
   *  \returns copy of the rotation vector which is unique
   */
  RotationVector getUnique() const {
    // todo: test, passive and active
    AngleAxis<PrimType_, Usage_> angleAxis(*this);
    RotationVector rotationVector(angleAxis.getUnique());
//    RotationVector rotVector(toImplementation());
//
//    const Scalar norm = rotVector.toImplementation().norm();
//    if (norm != Scalar(0)) {
//      const Scalar normWrapped = kindr::common::floatingPointModulo(norm+M_PI,2*M_PI)-M_PI;
//      rotVector.toImplementation()*normWrapped/norm;
//    }
    return rotationVector;
  }

  /*! \brief
   *  \returns reference
   */
  RotationVector& setUnique() {
    *this = getUnique();
    return *this;
  }

  /*! \brief Concatenation operator.
   *  This is explicitly specified, because Eigen provides also an operator*.
   *  \returns the concatenation of two rotations
   */
  using RotationVectorBase<RotationVector<PrimType_, Usage_>, Usage_>::operator*;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationVector& rotationVector) {
    out << rotationVector.toImplementation().transpose();
    return out;
  }
};

//! \brief Active rotation vector with double primitive type
typedef RotationVector<double, RotationUsage::ACTIVE>  RotationVectorAD;
//! \brief Active rotation vector with float primitive type
typedef RotationVector<float,  RotationUsage::ACTIVE>  RotationVectorAF;
//! \brief Passive rotation vector with double primitive type
typedef RotationVector<double, RotationUsage::PASSIVE> RotationVectorPD;
//! \brief Passive rotation vector with float primitive type
typedef RotationVector<float,  RotationUsage::PASSIVE> RotationVectorPF;


} // namespace eigen_impl


namespace internal {

template<typename PrimType_, enum RotationUsage Usage_>
class get_scalar<eigen_impl::RotationVector<PrimType_, Usage_>> {
 public:
  typedef PrimType_ Scalar;
};

template<typename PrimType_, enum RotationUsage Usage_>
class get_matrix3X<eigen_impl::RotationVector<PrimType_, Usage_>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_impl::RotationVector<PrimType_, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::RotationVector<PrimType_, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE> OtherUsage;
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationVector<DestPrimType_, Usage_>, eigen_impl::RotationVector<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationVector<DestPrimType_, Usage_> convert(const eigen_impl::RotationVector<SourcePrimType_, Usage_>& rotationVector) {
    return eigen_impl::RotationVector<DestPrimType_, Usage_>(rotationVector.toImplementation().template cast<DestPrimType_>());
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationVector<DestPrimType_, Usage_>, eigen_impl::AngleAxis<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationVector<DestPrimType_, Usage_> convert(const eigen_impl::AngleAxis<SourcePrimType_, Usage_>& angleAxis) {
    const eigen_impl::AngleAxis<DestPrimType_, Usage_> angleAxisDest(angleAxis);
    return eigen_impl::RotationVector<DestPrimType_, Usage_>(angleAxisDest.angle()*angleAxisDest.axis());
  }
};


template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationVector<DestPrimType_, Usage_>, eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationVector<DestPrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>& q) {
    if (1.0-q.real()*q.real() < common::internal::NumTraits<SourcePrimType_>::dummy_precision()) {
      return eigen_impl::RotationVector<DestPrimType_, Usage_>(2.0*q.imaginary().template cast<DestPrimType_>());
    }
    return eigen_impl::RotationVector<DestPrimType_, Usage_>((2.0*acos(q.real())/sqrt(1.0-q.real()*q.real())*q.imaginary()).template cast<DestPrimType_>());
  }
};

// Generic conversion
template<typename DestPrimType_, typename SourceImplementation_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationVector<DestPrimType_, Usage_>, SourceImplementation_> {
 public:
  inline static eigen_impl::RotationVector<DestPrimType_, Usage_> convert(const SourceImplementation_& rotation) {
    return eigen_impl::RotationVector<DestPrimType_, Usage_>(eigen_impl::AngleAxis<DestPrimType_, Usage_>(rotation));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationVector<DestPrimType_, Usage_>, eigen_impl::RotationMatrix<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationVector<DestPrimType_, Usage_> convert(const eigen_impl::RotationMatrix<SourcePrimType_, Usage_>& rotationMatrix) {
    // ok
    return eigen_impl::RotationVector<DestPrimType_, Usage_>(eigen_impl::AngleAxis<DestPrimType_, Usage_>(rotationMatrix));



//    typename eigen_impl::RotationMatrix<DestPrimType_, Usage_>::Implementation C = rotationMatrix.toImplementation().transpose().template cast<DestPrimType_>();
//    typename eigen_impl::RotationVector<DestPrimType_, Usage_>::Vector p;
//    // Sometimes, because of roundoff error, the value of tr ends up outside
//    // the valid range of arccos. Truncate to the valid range.
//    double tr = std::max(-1.0, std::min( (C(0,0) + C(1,1) + C(2,2) - 1.0) * 0.5, 1.0));
//    double a = acos( tr ) ;
//
//    if(fabs(a) < 1e-14){
//        return eigen_impl::RotationVector<DestPrimType_, Usage_>();
//    }
//
//    p[0] = (C(2,1) - C(1,2));
//    p[1] = (C(0,2) - C(2,0));
//    p[2] = (C(1,0) - C(0,1));
//    double n2 = p.norm();
//    if(fabs(n2) < 1e-14)
//      return eigen_impl::RotationVector<DestPrimType_, Usage_>();
//
//    double scale = -a/n2;
//    p = scale * p;
//
//
//      return eigen_impl::RotationVector<DestPrimType_, Usage_>(p);


  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Multiplication Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

///* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// * Comparison Traits
// * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
//template<typename PrimType_, enum RotationUsage Usage_>
//class ComparisonTraits<eigen_impl::RotationVector<PrimType_, Usage_>> {
// public:
//  inline static bool isEqual(const eigen_impl::RotationVector<PrimType_, Usage_>& a, const eigen_impl::RotationVector<PrimType_, Usage_>& b){
//    return a.toImplementation() ==  b.toImplementation();
//  }
//};

} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_EIGEN_ROTATIONVECTOR_HPP_ */
