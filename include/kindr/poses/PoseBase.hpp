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
#ifndef KINDR_POSES_POSESBASE_HPP_
#define KINDR_POSES_POSESBASE_HPP_


#include "kindr/common/common.hpp"


namespace kindr {
//! Generic pose interface
/*! \ingroup poses
 */
namespace poses {
//! Internal stuff (only for developers)
namespace internal {

/*! \class TransformationTraits
 * \brief Transformation trait to implement transformations of positions.
 *
 *  (only for advanced users)
 */
template<typename Pose_>
class TransformationTraits {
 public:
//  inline static Position transform(const Pose& pose, const Position& position);
//  inline static Position inverseTransform(const Pose& pose, const Position& position);
};

/*! \class get_position
 * \brief This class provides the type of the position.
 *
 *  (only for advanced users)
 */
template<typename Pose_>
class get_position {
 public:
//  typedef typename positions::eigen_impl::Position3D Position;
};

} // namespace internal


/*! \class TransformationBase
 * \brief Base class that defines the interface of a homogeneous transformation
 *
 * \tparam Derived_ the derived class that should implement the homogeneous transformation
 * \ingroup poses
 */

template<typename Derived_>
class TransformationBase {
 public:
  /*! \brief Default constructor.
   *
   *  Creates a pose with all position coordinates set to zero and an identity orientation.
   */
  TransformationBase() = default;

  /*! \brief Constructor from derived pose.
   *  This constructor has been deleted because the abstract class does not contain any data.
   */
  TransformationBase(const Derived_&) = delete; // on purpose!!

  /*! \brief Gets the derived pose.
   *
   *  (only for advanced users)
   *  \returns the derived pose
   */
  operator Derived_& () {
    return static_cast<Derived_&>(*this);
  }

  /*! \brief Gets the derived pose.
   *
   *  (only for advanced users)
   *  \returns the derived pose
   */
  operator const Derived_& () const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief Gets the derived pose.
   *
   *  (only for advanced users)
   *  \returns the derived pose
   */
  const Derived_& derived() const {
    return static_cast<const Derived_&>(*this);
  }


  /*! \brief Gets the derived pose.
   *
   *  (only for advanced users)
   *  \returns the derived pose
   */
  Derived_& derived() {
    return static_cast<Derived_&>(*this);
  }


  /*! \brief Transforms a position.
   *  \returns the transformed position
   */
  typename internal::get_position<Derived_>::Position transform(const typename internal::get_position<Derived_>::Position& position) const {
    return derived().getRotation().rotate(position) + derived().getPosition();
  }

  /*! \brief Transforms a position in reverse
   *  \returns the transformed position
   */
  typename internal::get_position<Derived_>::Position inverseTransform(const typename internal::get_position<Derived_>::Position& position) const {
    return derived().getRotation().inverseRotate((position-derived().getPosition()));
  }

  /*! \brief Sets the pose to identity
   *  \returns reference
   */
  Derived_& setIdentity(){ 
    derived().getPosition().setZero();
    derived().getRotation().setIdentity();
    return derived();
  }

  /*! \brief Returns the inverse of the pose
   *  \returns inverse of the pose
   */
  Derived_ inverted() const {
    return Derived_( -derived().getRotation().inverseRotate(derived().getPosition()), derived().getRotation().inverted() );
  }

  /*! \brief Inverts the pose
   *  \returns reference
   */
  Derived_& invert() {
    derived().getPosition() = -derived().getRotation().inverseRotate(derived().getPosition());
    derived().getRotation().invert();
    return *this;
  }

  /*! \brief Returns the pose in a unique form
   *  This function is used to compare different poses.
   *  \returns copy of the pose which is unique
   */
  Derived_ getUnique() const;

  /*! \brief  Modifies the pose such that it is in its unique form
   *  \returns reference
   */
  Derived_& setUnique();

  /*! \brief Concatenates two transformations.
   *  \returns the concatenation of two transformations
   */
  template<typename OtherDerived_>
  Derived_ operator *(const TransformationBase<OtherDerived_>& other) const {
    return Derived_(derived().getPosition() + derived().getRotation().rotate(other.derived().getPosition()), derived().getRotation() * other.derived().getRotation());
  }

  /*! \brief Compares two rotations.
   *  \returns true if the rotations are exactly equal
   */
  template<typename OtherDerived_>
  bool operator ==(const TransformationBase<OtherDerived_>& other) const {
    return (derived().getRotation() == other.derived().getRotation()) && (derived().getPosition() == other.derived().getPosition());
  }


};


} // namespace internal

namespace internal {

} // namespace poses
} // namespace kindr


#endif /* KINDR_POSES_POSESBASE_HPP_ */
