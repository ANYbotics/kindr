/*
 * QuaternionBase.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef QUATERNIONBASE_HPP_
#define QUATERNIONBASE_HPP_


namespace rm {
namespace quaternions {

namespace internal {
template<typename DEST, typename SOURCE>
class ConversionTraits
{
  // DEST convert(const SOURCE & );
};

template<typename LEFT, typename RIGHT>
class MultiplicationTraits {
  // LEFT mult(const LEFT &, const RIGHT & );
};

}

template<typename DERIVED>
class QuaternionBase {
 public:
  QuaternionBase inverse();
  QuaternionBase conjugate();

  operator DERIVED & () {
    return static_cast<DERIVED &>(*this);
  }
  operator const DERIVED & () const {
    return static_cast<const DERIVED &>(*this);
  }
};

}
}

#endif /* QUATERNIONBASE_HPP_ */
