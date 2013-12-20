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
  RotationVector(Scalar x, Scalar y, Scalar z) {
    if(Usage_ == RotationUsage::ACTIVE) {
      vector_ << x,y,z;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      vector_ << -x,-y,-z;
    }
  }


  /*! \brief Constructor using Eigen::Matrix<Scalar, 3, 1>.
   *
   *  \param other   Eigen::Matrix<Scalar, 3, 1>
   */
  explicit RotationVector(const Base& other) { // explicit on purpose
    if(Usage_ == RotationUsage::ACTIVE) {
      vector_ = other;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      vector_ = -other;
    }
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit RotationVector(const RotationBase<OtherDerived_, Usage_>& other)
    : vector_(internal::ConversionTraits<RotationVector, OtherDerived_>::convert(other.derived()).toStoredImplementation()) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationVector& operator =(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toStoredImplementation() = internal::ConversionTraits<RotationVector, OtherDerived_>::convert(other.derived()).toStoredImplementation();
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationVector& operator ()(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toStoredImplementation() = internal::ConversionTraits<RotationVector, OtherDerived_>::convert(other.derived()).toStoredImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  RotationVector inverted() const {
    return RotationVector(AngleAxis<PrimType_, Usage_>(this->toStoredImplementation().norm(), this->toStoredImplementation().normalized()).inverse());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  RotationVector& invert() {
    *this = inverted();
    return *this;
  }

  /*! \brief Returns the type used for the implementation.
   *  \returns the type used for the implementation
   */
  Implementation toImplementation() const {
    if(Usage_ == RotationUsage::PASSIVE) {
      return -vector_;
    }
    return vector_;
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation& toStoredImplementation() {
    return static_cast<Implementation&>(vector_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation& toStoredImplementation() const {
    return static_cast<const Implementation&>(vector_);
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  RotationVector& setIdentity() {
    this->toImplementatoStoredImplementation;
    return *this;
  }

  /*! \brief Returns the rotation vector.
   *  \returns the rotation vector (scalar)
   */
  inline const Implementation vector() const {
    return this->toImplementation();
  }

  /*! \brief Sets the rotation vector.
   * \param x   first entry
   * \param y   second entry
   * \param z   third entry
   */
  inline void setVector(Scalar x, Scalar y, Scalar z) {
    if(Usage_ == RotationUsage::ACTIVE) {
      vector_ << x, y, z;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      vector_ << -x, -y, -z;
    }
  }

  /*! \brief Sets the rotation vector.
   */
  inline void setVector(const Implementation& vector) {
    if(Usage_ == RotationUsage::ACTIVE) {
      vector_ = vector;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      vector_ = -vector;
    }
  }

  /*! \brief Returns the first entry of the rotation vector.
   *  \returns first entry of the rotation vector (scalar)
   */
  inline Scalar x() const {
    if(Usage_ == RotationUsage::PASSIVE) {
      return -vector_(0);
    }
    return vector_(0);
  }

  /*! \brief Returns the second entry of the rotation vector.
   *  \returns second entry of the rotation vector (scalar)
   */
  inline Scalar y() const {
    if(Usage_ == RotationUsage::PASSIVE) {
      return -vector_(1);
    }
    return vector_(1);
  }

  /*! \brief Returns the third entry of the rotation vector.
   *  \returns third entry of the rotation vector (scalar)
   */
  inline Scalar z() const {
    if(Usage_ == RotationUsage::PASSIVE) {
      return -vector_(2);
    }
    return vector_(2);
  }

  /*! \brief Sets the first entry of the rotation vector.
   */
  inline void setX(Scalar x) {
    if(Usage_ == RotationUsage::ACTIVE) {
      vector_(0) = x;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      vector_(0) = -x;
    }
  }

  /*! \brief Sets the second entry of the rotation vector.
   */
  inline void setY(Scalar y) {
    if(Usage_ == RotationUsage::ACTIVE) {
      vector_(1) = y;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      vector_(1) = -y;
    }
  }

  /*! \brief Sets the third entry of the rotation vector.
   */
  inline void setZ(Scalar z) {
    if(Usage_ == RotationUsage::ACTIVE) {
      vector_(2) = z;
    } else if(Usage_ == RotationUsage::PASSIVE) {
      vector_(2) = -z;
    }
  }

  /*! \brief Returns a unique rotation vector with norm in [0,pi).
   *  This function is used to compare different rotations.
   *  \returns copy of the rotation vector which is unique
   */
  RotationVector getUnique() const {
    // todo: test, passive and active
    RotationVector rotVector(toStoredImplementation());

    const Scalar norm = rotVector.toStoredImplementation().norm();
    if (norm != Scalar(0)) {
      const Scalar normWrapped = kindr::common::floatingPointModulo(norm+M_PI,2*M_PI)-M_PI;
      rotVector.toStoredImplementation()*normWrapped/norm;
    }
    return rotVector;
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
    return eigen_impl::RotationVector<DestPrimType_, Usage_>(rotationVector.toStoredImplementation().template cast<DestPrimType_>());
  }
};


template<typename DestPrimType_, typename SourceImplementation_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationVector<DestPrimType_, Usage_>, SourceImplementation_> {
 public:
  inline static eigen_impl::RotationVector<DestPrimType_, Usage_> convert(const SourceImplementation_& rotation) {
    const eigen_impl::AngleAxis<DestPrimType_, Usage_> angleAxis(rotation);
    return eigen_impl::RotationVector<DestPrimType_, Usage_>(angleAxis.angle()*angleAxis.axis());
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Multiplication Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename PrimType_, enum RotationUsage Usage_>
class RotationTraits<eigen_impl::RotationVector<PrimType_, Usage_>> {
 public:
  template<typename get_matrix3X<eigen_impl::RotationVector<PrimType_, Usage_>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_impl::RotationVector<PrimType_, Usage_>>::template Matrix3X<Cols> rotate(const eigen_impl::RotationVector<PrimType_, Usage_>& rv, const typename get_matrix3X<eigen_impl::RotationVector<PrimType_, Usage_>>::template Matrix3X<Cols>& m){
    return eigen_impl::RotationMatrix<PrimType_, Usage_>(rv).toStoredImplementation() * m;
  }
};

///* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// * Comparison Traits
// * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
//template<typename PrimType_, enum RotationUsage Usage_>
//class ComparisonTraits<eigen_impl::RotationVector<PrimType_, Usage_>> {
// public:
//  inline static bool isEqual(const eigen_impl::RotationVector<PrimType_, Usage_>& a, const eigen_impl::RotationVector<PrimType_, Usage_>& b){
//    return a.toStoredImplementation() ==  b.toStoredImplementation();
//  }
//};

} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_EIGEN_ROTATIONVECTOR_HPP_ */
