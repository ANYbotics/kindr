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

#ifndef KINDR_PHYSICALQUANTITIES_VECTOR_HPP_
#define KINDR_PHYSICALQUANTITIES_VECTOR_HPP_

#include <Eigen/Core>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/vector/VectorBase.hpp"

namespace kindr {
namespace vector {
//! Implementation of rotations based on the C++ Eigen library
namespace eigen_impl {

/*! \class Vector3
 * \brief Vector in 3D-space.
 *
 * This class implements a vector in 3D-space.
 * More precisely an interface to store and access the coordinates of a vector of a point in 3D-space is provided.
 * \tparam PrimType_  Primitive type of the coordinates.
 * \ingroup vectors
 */
template<typename PrimType_, int Dimension_>
class Vector : public VectorBase<Vector<PrimType_, Dimension_> >, private Eigen::Matrix<PrimType_, Dimension_, 1> {
 private:
  /*! \brief The base type.
   */
  typedef VectorBase<Vector<PrimType_, Dimension_> > Base;
 public:
  /*! \brief The implementation type.
   *
   *  The implementation type is always an Eigen object.
   */
  typedef Eigen::Matrix<PrimType_, Dimension_, 1> Implementation;

  /*! \brief The primitive type of the coordinates.
   */
  typedef PrimType_ Scalar;

  /*! \brief The dimension of the vector.
   */
  static constexpr int Dimension = Dimension_;

  /*! \brief Default constructor initializes all coordinates of the vector with zero.
   */
  Vector()
    : Implementation(Implementation::Zero()) {
  }

  /*! \brief Default constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,Dimension_,1>
   */
  explicit Vector(const Implementation & other)
    : Implementation(other) {
  }

  /*! \brief Operator () using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,Dimension_,1>
   */
  Vector<PrimType_, Dimension_>& operator ()(const Implementation& other) {
    this->toImplementation() = other;
    return *this;
  }

  /*! \brief Set values.
   */
  using Implementation::operator<<;

  /*! \brief Get/set values.
   */
  using Implementation::operator();

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return static_cast<const Implementation&>(*this);
  }

  /*! \brief Addition of two vectors.
   */
  using VectorBase<Vector<PrimType_, Dimension_>>::operator+; // otherwise ambiguous VectorBase and Eigen

  /*! \brief Subtraction of two vectors.
   */
  using VectorBase<Vector<PrimType_, Dimension_>>::operator-; // otherwise ambiguous VectorBase and Eigen

  /*! \brief Addition of two vectors.
   * \param other   other vector
   */
  template<typename Other_>
  Vector<PrimType_, Dimension_>& operator +=(const Other_& other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  /*! \brief Subtraction of two vectors.
   * \param other   other vector
   */
  template<typename Other_>
  Vector<PrimType_, Dimension_>& operator -=(const Other_& other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  /*! \brief Sets all coordinates of the vector to zero.
   * \returns reference
   */
  Vector<PrimType_, Dimension_>& setZero() {
    Implementation::setZero();
    return *this;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const Vector<PrimType_, Dimension_>& vector) {
    out << vector.transpose();
    return out;
  }
};




/*! \class Vector3
 * \brief Vector in 3D-space.
 *
 * This class implements a vector in 3D-space.
 * More precisely an interface to store and access the coordinates of a vector of a point in 3D-space is provided.
 * \tparam PrimType_  Primitive type of the coordinates.
 * \ingroup vectors
 */
template<typename PrimType_>
class Vector3 : public Vector3Base<Vector3<PrimType_> >, private Eigen::Matrix<PrimType_, 3, 1> {
 private:
  /*! \brief The base type.
   */
  typedef Vector3Base<Vector3<PrimType_> > Base;
 public:
  /*! \brief The implementation type.
   *
   *  The implementation type is always an Eigen object.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Implementation;

  /*! \brief The primitive type of the coordinates.
   */
  typedef PrimType_ Scalar;

  /*! \brief The dimension of the vector.
   */
  static constexpr int Dimension = 3;

  /*! \brief Default constructor initializes all coordinates of the vector with zero.
   */
  Vector3()
    : Implementation(Implementation::Zero()) {
  }

  /*! Constructor with three coordinates (x,y,z)
   * \param x   x-coordinate
   * \param y   y-coordinate
   * \param z   z-coordinate
   */
  Vector3(Scalar x, Scalar y, Scalar z)
    : Implementation(x, y, z) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,3,1>
   */
  explicit Vector3(const Implementation& other)
    : Implementation(other) {
  }

  /*! \brief Operator () using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,Dimension_,1>
   */
  Vector3<PrimType_>& operator ()(const Implementation& other) {
    this->toImplementation() = other;
    return *this;
  }

  /*! \brief Set values.
   */
  using Implementation::operator<<;

  /*! \brief Get/set values.
   */
  using Implementation::operator();

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return static_cast<const Implementation&>(*this);
  }

  /*!\brief Get x-coordinate of the vector
   * \returns the x-coordinate of the vector
   */
  using Implementation::x;

  /*!\brief Get y-coordinate of the vector
   * \returns the y-coordinate of the vector
   */
  using Implementation::y;

  /*!\brief Get z-coordinate of the vector
   * \returns the z-coordinate of the vector
   */
  using Implementation::z;

  /*! \brief Addition of two vectors.
   */
  using Vector3Base<Vector3<PrimType_>>::operator+; // otherwise ambiguous VectorBase and Eigen

  /*! \brief Subtraction of two vectors.
   */
  using Vector3Base<Vector3<PrimType_>>::operator-; // otherwise ambiguous VectorBase and Eigen

  /*! \brief Addition of two vectors.
   * \param other   other vector
   */
  template<typename Other_>
  Vector3<PrimType_>& operator +=(const Other_& other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  /*! \brief Subtraction of two vectors.
   * \param other   other vector
   */
  template<typename Other_>
  Vector3<PrimType_>& operator -=(const Other_& other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  /*! \brief Sets all coordinates of the vector to zero.
   * \returns reference
   */
  Vector3<PrimType_>& setZero() {
    Implementation::setZero();
    return *this;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const Vector3& vector) {
    out << vector.transpose();
    return out;
  }
};


//! \brief 3D-Vector with primitive type double
typedef Vector3<double>  Vector3D;

//! \brief 3D-Vector with primitive type float
typedef Vector3<float>  Vector3F;

} // namespace eigen_impl



namespace internal {

/*! \brief Gets the primitive type of the vector
 */
template<typename PrimType_, int Dimension_>
class get_scalar<eigen_impl::Vector<PrimType_, Dimension_>>{
 public:
  typedef PrimType_ Scalar;
};
template<typename PrimType_>
class get_scalar<eigen_impl::Vector3<PrimType_>>{
 public:
  typedef PrimType_ Scalar;
};


/*! \brief Gets the dimension of the vector
 */
template<typename PrimType_, int Dimension_>
class get_dimension<eigen_impl::Vector<PrimType_, Dimension_>>{
 public:
  static constexpr int Dimension = Dimension_;
};
template<typename PrimType_>
class get_dimension<eigen_impl::Vector3<PrimType_>>{
 public:
  static constexpr int Dimension = 3;
};



} // namespace internal
} // namespace vector
} // namespace kindr





#endif /* KINDR_PHYSICALQUANTITIES_VECTOR_HPP_ */
