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
 * Wernch.hpp
 *
 *  Created on: Jan 31, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <kindr/phys_quant/WrenchBase.hpp>
#include <kindr/phys_quant/PhysicalQuantities.hpp>

namespace kindr {

//!
template <typename PrimType_>
class Wrench6 : public WrenchBase<Wrench6<PrimType_>> {
public:
  typedef PrimType_ Scalar;
  typedef Vector<PhysicalType::Force, PrimType_, 3> Force;
  typedef Vector<PhysicalType::Torque, PrimType_, 3> Torque;
  typedef Eigen::Matrix<PrimType_, 6, 1> Vector6;
  typedef Eigen::Matrix<PrimType_, 3, 1> Vector3;

  explicit Wrench6() : force_(Force::Zero()),  torque_(Torque::Zero()) {

  }

  explicit Wrench6(const Force& force, const Torque& torque) :
    force_(force),
    torque_(torque) {
  }

  explicit Wrench6(const Vector3& force, const Vector3& torque) :
    force_(Force(force)),
    torque_(Torque(torque)) {
  }

  explicit Wrench6(const Vector6& wrench) :
    force_(Force(wrench.head(3))),
    torque_(Torque(wrench.tail(3))) {
  }

  virtual ~Wrench6() {

  }

  inline Force & getForce() {
    return force_;
  }

  inline const Force & getForce() const {
    return force_;
  }

  inline Torque & getTorque() {
    return torque_;
  }

  inline const Torque & getTorque() const {
    return torque_;
  }

  inline void setForce(const Force& force) {
    force_ = force;
  }

  inline void setTorque(const Torque& torque) {
    torque_ = torque;
  }

  inline void setForce(const Vector3& force) {
    force_ = Force(force);
  }

  inline  void setTorque(const Vector3& torque) {
    torque_ = Torque(torque);
  }

  inline void setVector(const Vector6& wrench) {
    *this = Wrench6(wrench);
  }

  inline Vector6 getVector() const {
    Vector6 vector;
    vector.template head<3>() = getForce().toImplementation();
    vector.template tail<3>() = getTorque().toImplementation();
    return vector;
  }

  Wrench6& setZero() {
    force_.setZero();
    torque_.setZero();
    return *this;
  }

  /*! \brief Assignment operator.
     * \param other   other vector
     * \returns reference
     */
  Wrench6 & operator=(const Wrench6& other) {
    this->getForce() = other.getForce();
    this->getTorque() = other.getTorque();
    return *this;
  }

  /*! \brief Addition of two vectors.
   * \param other   other vector
   * \returns sum
   */
  Wrench6 operator+(const Wrench6& other) const {
    return Wrench6(this->getForce() + other.getForce(), this->getTorque() + other.getTorque());
  }

  /*! \brief Subtraction of two vectors.
   * \param other   other vector
   * \returns difference
   */
  Wrench6 operator-(const Wrench6& other) const {
    return Wrench6(this->getForce() - other.getForce(), this->getTorque() - other.getTorque());
  }

  /*! \brief Multiplies vector with a scalar.
   * \param factor   factor
   * \returns product
   */
  template<typename PrimTypeFactor_>
  Wrench6 operator*(PrimTypeFactor_ factor) const {
    return Wrench6(this->getForce()*(PrimType_)factor, this->getTorque()*(PrimType_)factor);
  }

  /*! \brief Divides vector by a scalar.
   * \param divisor   divisor
   * \returns quotient
   */
  template<typename PrimTypeDivisor_>
  Wrench6 operator/(PrimTypeDivisor_ divisor) const {
    return Wrench6(this->getForce()/(PrimType_)divisor, this->getTorque()/(PrimType_)divisor);
  }

  /*! \brief Addition and assignment of two vectors.
   * \param other   other vector
   * \returns reference
   */
  Wrench6& operator+=(const Wrench6& other) {
    this->getForce() += other.getForce();
    this->getTorque() += other.getTorque();
    return *this;
  }

  /*! \brief Subtraction and assignment of two vectors.
   * \param other   other vector
   * \returns reference
   */
  Wrench6& operator-=(const Wrench6& other) {
    this->getForce() -= other.getForce();
    this->getTorque() -= other.getTorque();
    return *this;
  }

  /*! \brief Multiplication with a scalar and assignment.
   * \param factor   factor
   * \returns reference
   */
  template<typename PrimTypeFactor_>
  Wrench6& operator*=(PrimTypeFactor_ factor) {
    this->getForce() *= (PrimType_)factor;
    this->getTorque() *= (PrimType_)factor;
    return *this;
  }

  /*! \brief Division by a scalar and assignment.
   * \param divisor   divisor
   * \returns reference
   */
  template<typename PrimTypeDivisor_>
  Wrench6& operator/=(PrimTypeDivisor_ divisor) {
    this->getForce() /= (PrimType_)divisor;
    this->getTorque() /= (PrimType_)divisor;
    return *this;
  }

  /*! \brief Negation of a vector.
   * \returns negative vector
   */
  Wrench6 operator-() const {
    return Wrench6(-this->getForce(), -this->getTorque());
  }

  /*! \brief Comparison operator.
   * \param other   other vector
   * \returns true if equal
   */
  bool operator==(const Wrench6& other) const {
    return ((this->getForce() == other.getForce()) && (this->getTorque() == other.getTorque()));
  }


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const Wrench6 & wrench) {
    out << wrench.getForce() << " " << wrench.getTorque();
    return out;
  }
protected:
  Force force_;
  Torque torque_;
};

typedef Wrench6<double> WrenchD;
typedef Wrench6<float> WrenchF;


} // namespace kindr
