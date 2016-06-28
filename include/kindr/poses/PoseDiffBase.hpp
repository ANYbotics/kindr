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

namespace kindr {

/*! \class PoseDiffBase
 * \brief Base class that defines the interface of the time derivative of a pose of a rigid body.
 *
 *  This class defines the interface of the time derivative of a pose of a rigid body.
 *  The time derivative of a pose of a rigid body describes the linear and angular velocities of the rigid body in space.
 * \tparam Derived_ the derived class that should implement the time derivative of the pose.
 * \ingroup poses
 */
template<typename Derived_>
class PoseDiffBase {
 public:
  /*! \brief Default constructor.
   *
   *  Creates a time derivative of a pose with all derivatives set to zero.
   */
  PoseDiffBase() = default;

  /*! \brief Constructor from derived time derivative of a pose.
   *  This constructor has been deleted because the abstract class does not contain any data.
   */
  PoseDiffBase(const Derived_&) = delete; // on purpose!!

  /*! \brief Gets the derived time derivative of a pose.
   *
   *  (only for advanced users)
   *  \returns the derived time derivative of a pose
   */
  operator Derived_& () {
    return static_cast<Derived_&>(*this);
  }

  /*! \brief Gets the derived time derivative of a pose.
   *
   *  (only for advanced users)
   *  \returns the derived time derivative of a pose
   */
  operator const Derived_& () const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief Gets the derived time derivative of a pose.
   *
   *  (only for advanced users)
   *  \returns the derived time derivative of a pose
   */
  const Derived_& derived() const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief Sets twist to zero
   *  \returns reference
   */
  Derived_& setZero();
};


} // namespace kindr
