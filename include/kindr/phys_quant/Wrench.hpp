/*
 * Copyright (c) 2017, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
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
/*
 * Wrench.hpp
 *
 *  Created on: Jan 31, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <kindr/phys_quant/WrenchBase.hpp>
#include <kindr/phys_quant/PhysicalQuantities.hpp>

namespace kindr {

//! Wrench, i.e. generalized 6D force in screw algebra.
template <typename PrimType_>
class Wrench : public WrenchBase<Wrench<PrimType_>> {
 public:
  typedef PrimType_ Scalar;
  typedef Vector<PhysicalType::Force, PrimType_, 3> Force;
  typedef Vector<PhysicalType::Torque, PrimType_, 3> Torque;
  typedef Eigen::Matrix<PrimType_, 6, 1> Vector6;
  typedef Eigen::Matrix<PrimType_, 3, 1> Vector3;

  explicit Wrench() : force_(Force::Zero()),  torque_(Torque::Zero()) {

  }

  explicit Wrench(const Force& force, const Torque& torque) :
    force_(force),
    torque_(torque) {
  }

  explicit Wrench(const Vector3& force, const Vector3& torque) :
    force_(Force(force)),
    torque_(Torque(torque)) {
  }

  explicit Wrench(const Vector6& wrench) :
    force_(Force(wrench.head(3))),
    torque_(Torque(wrench.tail(3))) {
  }

  virtual ~Wrench() = default;

  //! Get resultant force of the wrench
  inline Force & getForce() {
    return force_;
  }

  //! Get resultant force of the wrench
  inline const Force & getForce() const {
    return force_;
  }

  //! Get resultant moment (a.k.a. torque) of the wrench
  inline Torque & getTorque() {
    return torque_;
  }

  //! Get resultant moment (a.k.a. torque) of the wrench
  inline const Torque & getTorque() const {
    return torque_;
  }

  //! Set resultant force of the wrench
  inline void setForce(const Force& force) {
    force_ = force;
  }

  //! Set resultant moment (a.k.a. torque) of the wrench
  inline void setTorque(const Torque& torque) {
    torque_ = torque;
  }

  //! Set resultant force of the wrench
  inline void setForce(const Vector3& force) {
    force_ = Force(force);
  }

  //! Set resultant moment (a.k.a. torque) of the wrench
  inline  void setTorque(const Vector3& torque) {
    torque_ = Torque(torque);
  }

  //! Set both resultant force and moment of the wrench from a 6D vector
  inline void setVector(const Vector6& wrench) {
    *this = Wrench(wrench);
  }

  //! Get 6D vector representation of the wrench (force before torque)
  inline Vector6 getVector() const {
    Vector6 vector;
    vector.template head<3>() = getForce().toImplementation();
    vector.template tail<3>() = getTorque().toImplementation();
    return vector;
  }

  /*! \brief Set wrench to zero
   *  \returns reference
   */
  Wrench& setZero() {
    force_.setZero();
    torque_.setZero();
    return *this;
  }

  /*! \brief Get zero wrench
   *  \returns zero wrench
   */
  static Wrench Zero() {
    return Wrench(Force::Zero(), Torque::Zero());
  }

  /*! \brief Assignment operator.
     * \param other   other vector
     * \returns reference
     */
  Wrench & operator=(const Wrench& other) = default;

  /*! \brief Addition of two vectors.
   * \param other   other vector
   * \returns sum
   */
  Wrench operator+(const Wrench& other) const {
    return Wrench(this->getForce() + other.getForce(), this->getTorque() + other.getTorque());
  }

  /*! \brief Subtraction of two vectors.
   * \param other   other vector
   * \returns difference
   */
  Wrench operator-(const Wrench& other) const {
    return Wrench(this->getForce() - other.getForce(), this->getTorque() - other.getTorque());
  }

  /*! \brief Multiplies vector with a scalar.
   * \param factor   factor
   * \returns product
   */
  template<typename PrimTypeFactor_>
  Wrench operator*(PrimTypeFactor_ factor) const {
    return Wrench(this->getForce()*(PrimType_)factor, this->getTorque()*(PrimType_)factor);
  }

  /*! \brief Divides vector by a scalar.
   * \param divisor   divisor
   * \returns quotient
   */
  template<typename PrimTypeDivisor_>
  Wrench operator/(PrimTypeDivisor_ divisor) const {
    return Wrench(this->getForce()/(PrimType_)divisor, this->getTorque()/(PrimType_)divisor);
  }

  /*! \brief Addition and assignment of two vectors.
   * \param other   other vector
   * \returns reference
   */
  Wrench& operator+=(const Wrench& other) {
    this->getForce() += other.getForce();
    this->getTorque() += other.getTorque();
    return *this;
  }

  /*! \brief Subtraction and assignment of two vectors.
   * \param other   other vector
   * \returns reference
   */
  Wrench& operator-=(const Wrench& other) {
    this->getForce() -= other.getForce();
    this->getTorque() -= other.getTorque();
    return *this;
  }

  /*! \brief Multiplication with a scalar and assignment.
   * \param factor   factor
   * \returns reference
   */
  template<typename PrimTypeFactor_>
  Wrench& operator*=(PrimTypeFactor_ factor) {
    this->getForce() *= (PrimType_)factor;
    this->getTorque() *= (PrimType_)factor;
    return *this;
  }

  /*! \brief Division by a scalar and assignment.
   * \param divisor   divisor
   * \returns reference
   */
  template<typename PrimTypeDivisor_>
  Wrench& operator/=(PrimTypeDivisor_ divisor) {
    this->getForce() /= (PrimType_)divisor;
    this->getTorque() /= (PrimType_)divisor;
    return *this;
  }

  /*! \brief Negation of a vector.
   * \returns negative vector
   */
  Wrench operator-() const {
    return Wrench(-this->getForce(), -this->getTorque());
  }

  /*! \brief Comparison operator.
   * \param other   other vector
   * \returns true if equal
   */
  bool operator==(const Wrench& other) const {
    return ((this->getForce() == other.getForce()) && (this->getTorque() == other.getTorque()));
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const Wrench & wrench) {
    out << wrench.getForce() << " " << wrench.getTorque();
    return out;
  }

protected:
  //! Resultant force of the wrench
  Force force_;

  //! Resultant moment (a.k.a. torque) of the wrench
  Torque torque_;
};

typedef Wrench<double> WrenchD;
typedef Wrench<float> WrenchF;

// API backward compatibility
template <typename PrimType_>
using Wrench6 = Wrench<PrimType_>;

} // namespace kindr
