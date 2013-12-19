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
#ifndef KINDR_POSITIONS_PDIFFEIGEN_HPP_
#define KINDR_POSITIONS_PDIFFEIGEN_HPP_

#include <Eigen/Core>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/positions/PositionDiffBase.hpp"

namespace kindr {
namespace positions {
namespace eigen_impl {

/*! \class LinearVelocity3
 * \brief Linear velocity in 3D-space.
 *
 * This class implements a lienar velocity in 3D-space.
 * More precisely an interface to store and access the components of a linear velocity of a point in 3D-space is provided.
 * \tparam PrimType_  Primitive type of the coordinates.
 * \ingroup positions
 */
template<typename PrimType_>
class LinearVelocity3 : public LinearVelocity3Base<LinearVelocity3<PrimType_>>, private Eigen::Matrix<PrimType_, 3, 1> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;
 public:
  /*! \brief The implementation type.
   *
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;

  /*! \brief The primitive type of the coordinates.
   */
  typedef PrimType_ Scalar;

  /*! \brief Default constructor initializes the linear velocity with zero.
   */
  LinearVelocity3()
    : Base(Base::Zero()) {
  }

  /*! Constructor with three coordinates (x,y,z)
   * \param x   x-coordinate
   * \param y   y-coordinate
   * \param z   z-coordinate
   */
  LinearVelocity3(Scalar x, Scalar y, Scalar z)
    : Base(x, y, z) {
  }


  /*! \brief Constructor using Eigen::Vector3.
   *  \param other   Eigen::Matrix<PrimType_,3,1>
   */
  explicit LinearVelocity3(const Base& other)
    : Base(other) {
   }

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

  /*!\brief Get x-coordinate of the linear velocity
   * \returns the x-coordinate of the linear velocity
   */
  using Base::x;

  /*!\brief Get y-coordinate of the linear velocity
   * \returns the y-coordinate of the linear velocity
   */
  using Base::y;

  /*!\brief Get z-coordinate of the linear velocity
   * \returns the z-coordinate of the linear velocity
   */
  using Base::z;

  /*! \brief Addition of two linear velocities.
   */
  using LinearVelocity3Base<LinearVelocity3<PrimType_>>::operator+; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Subtraction of two linear velocities.
   */
  using LinearVelocity3Base<LinearVelocity3<PrimType_>>::operator-; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Addition of two linear velocities.
   * \param other   other linear velocity
   */
  template<typename Other_>
  LinearVelocity3<PrimType_>& operator +=(const Other_& other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  /*! \brief Subtraction of two linear velocities.
   * \param other   other linear velocity
   */
  template<typename Other_>
  LinearVelocity3<PrimType_>& operator -=(const Other_& other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  /*! \brief Sets all components of the velocity to zero.
   * \returns reference
   */
  LinearVelocity3<PrimType_>& setZero() {
    Base::setZero();
    return *this;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const LinearVelocity3& velocity) {
    out << velocity.transpose();
    return out;
  }
};


//! \brief Linear velocity in 3D space with primitive type double
typedef LinearVelocity3<double>  LinearVelocity3D;

//! \brief Linear velocity in 3D space with primitive type float
typedef LinearVelocity3<float>  LinearVelocity3F;

} // namespace eigen_impl

namespace internal {

/*! \brief Gets the primitive type of the coordinates
 */
template<typename PrimType_>
class get_scalar<eigen_impl::LinearVelocity3<PrimType_>>{
 public:
  typedef PrimType_ Scalar;
};



} // namespace internal
} // namespace positions
} // namespace kindr

#endif /* KINDR_POSITIONS_PDIFFEIGEN_HPP_ */
