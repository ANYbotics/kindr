/*
 * SO3.hpp
 *
 *  Created on: Nov 17, 2014
 *      Author: hannes
 */

#ifndef INCLUDE_KINDR_CORE_SO3_HPP_
#define INCLUDE_KINDR_CORE_SO3_HPP_

#include "SE3.hpp"

namespace kindr {
namespace core {
namespace oo {

template<class Derived>
class SO3 : public SE3<Derived> {
 public:
  typedef SE3<Derived> Base;
  using Base::Base;

  template <typename Other>
  Derived & operator = (const SO3<Other> & other){ Base::operator = (other); return *this; }
};

} // namespace oo
} // namespace core
} // namespace kindr

#endif /* INCLUDE_KINDR_CORE_SO3_HPP_ */
