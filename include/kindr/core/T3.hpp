/*
 * T3.hpp
 *
 *  Created on: Nov 17, 2014
 *      Author: hannes
 */

#ifndef INCLUDE_KINDR_CORE_T3_HPP_
#define INCLUDE_KINDR_CORE_T3_HPP_

#include "SE3.hpp"

namespace kindr {
namespace core {

template <typename Scalar_>
class T3 : public SE3< T3<Scalar_ >>{
 public:
  typedef SE3< T3<Scalar_> > Base;
  using Base::Base;
};

template<typename Scalar_>
struct ParamTraits< T3<Scalar_> > {
  typedef Scalar_ Scalar;
  typedef Vector3<Scalar> Storage;
  typedef Vector3<Scalar> LieAlgebraVector;

  static Vector3<Scalar> apply(const Storage & t, const Vector3<Scalar> & vec){
    return t + vec;
  }
  static Storage exp(const LieAlgebraVector & v){
    return v;
  }
};

typedef T3<float> Translation3F;
typedef T3<double> Translation3D;

}
}


#endif /* INCLUDE_KINDR_CORE_T3_HPP_ */
