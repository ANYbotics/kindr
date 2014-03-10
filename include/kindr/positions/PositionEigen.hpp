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

#ifndef KINDR_POSITIONS_POSITIONEIGEN_HPP_
#define KINDR_POSITIONS_POSITIONEIGEN_HPP_


#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/positions/PositionBase.hpp"
#include "kindr/vector/VectorEigen.hpp"

namespace kindr {
namespace positions {
//! Implementation of positions based on the C++ Eigen library
namespace eigen_impl {

/*! \class Position3
 * \brief Position in 3D-space.
 *
 * This class implements a position in 3D-space.
 * More precisely an interface to store and access the coordinates of a position of a point in 3D-space is provided.
 * \tparam PrimType_  Primitive type of the coordinates.
 * \ingroup positions
 */
template<typename PrimType_>
class Position3 : public Position3Base<Position3<PrimType_>>, public vector::eigen_impl::Vector<phys_quant::PhysicalType::Length, PrimType_, 3> {
 private:
  /*! \brief The base type.
   */
  typedef vector::eigen_impl::Vector<phys_quant::PhysicalType::Length, PrimType_, 3> Base;
 public:
  /*! \brief The implementation type.
   *
   *  The implementation type is always an Eigen object.
   */
  typedef typename Base::Implementation Implementation;

  /*! \brief The primitive type of the coordinates.
   */
  typedef PrimType_ Scalar;

  /*! \brief Default constructor initializes all coordinates of the position with zero.
   */
  Position3()
    : Base(Base::Zero()) {
  }

  /*! Constructor with three coordinates (x,y,z)
   * \param x   x-coordinate
   * \param y   y-coordinate
   * \param z   z-coordinate
   */
  Position3(Scalar x, Scalar y, Scalar z)
    : Base(x, y, z) {
  }

  /*! \brief Constructor using Vector.
   *  \param other   vector::eigen_impl::Vector<phys_quant::PhysicalType::Length, PrimType_, 3>
   */
  explicit Position3(const Base& other)
    : Base(other) {
  }

  /*! \brief Constructor using Eigen::Vector3.
   *  \param other   Eigen::Matrix<PrimType_,3,1>
   */
  explicit Position3(const Implementation& other)
    : Base(other) {
  }

  /*! \brief Cast to the base type.
   *  \returns the base (recommended only for advanced users)
   */
  inline Base& toBase() {
    return static_cast<Base&>(*this);
  }

  /*! \brief Cast to the base type.
   *  \returns the base (recommended only for advanced users)
   */
  inline const Base& toBase() const {
    return static_cast<const Base&>(*this);
  }

  /*!\brief Get x-coordinate of the position
   * \returns the x-coordinate of the position
   */
  using Base::x;

  /*!\brief Get y-coordinate of the position
   * \returns the y-coordinate of the position
   */
  using Base::y;

  /*!\brief Get z-coordinate of the position
   * \returns the z-coordinate of the position
   */
  using Base::z;

  /*! \brief Addition of two positions.
   */
  using Position3Base<Position3<PrimType_>>::operator+; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Subtraction of two positions.
   */
  using Position3Base<Position3<PrimType_>>::operator-; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Multiplication of a position and a factor.
   */
  using Position3Base<Position3<PrimType_>>::operator*; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Division of a position by a factor.
   */
  using Position3Base<Position3<PrimType_>>::operator/; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Addition and assignment.
   * \param other   other position
   * \returns reference
   */
  template<typename Other_>
  Position3<PrimType_>& operator +=(const Other_& other) {
    this->toBase() += other.toBase();
    return *this;
  }

  /*! \brief Subtraction and assignment.
   * \param other   other position
   * \returns reference
   */
  template<typename Other_>
  Position3<PrimType_>& operator -=(const Other_& other) {
    this->toBase() -= other.toBase();
    return *this;
  }

  /*! \brief Multiplication and assignment.
   * \param factor   factor
   * \returns reference
   */
  template<typename FactorPrimType_>
  Position3<PrimType_>& operator *=(FactorPrimType_ factor) {
    this->toBase() *= (PrimType_)factor;
    return *this;
  }

  /*! \brief Division and assignment.
   * \param divisor   divisor
   * \returns reference
   */
  template<typename DivisorPrimType_>
  Position3<PrimType_>& operator /=(DivisorPrimType_ divisor) {
    this->toBase() /= (PrimType_)divisor;
    return *this;
  }
};


/*! \brief Multiplies a position with a scalar.
 * \param factor   factor
 * \returns product
 */
template<typename FactorPrimType_, typename PrimType_>
Position3<PrimType_> operator*(FactorPrimType_ factor, const Position3<PrimType_>& position) {
  return position*(PrimType_)factor;
}


//! \brief 3D-Position with primitive type double
typedef Position3<double>  Position3D;

//! \brief 3D-Position with primitive type float
typedef Position3<float>  Position3F;

} // namespace eigen_impl

namespace internal {

/*! \brief Gets the primitive type of the coordinates
 */
template<typename PrimType_>
class get_scalar<eigen_impl::Position3<PrimType_>> {
 public:
  typedef PrimType_ Scalar;
};



} // namespace internal
} // namespace positions
} // namespace kindr





#endif /* KINDR_POSITIONS_POSITIONEIGEN_HPP_ */
