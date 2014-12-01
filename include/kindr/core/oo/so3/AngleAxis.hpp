/*
 * AngleAxis.hpp
 *
 *  Created on: Nov 30, 2014
 *      Author: hannes
 */


#ifndef ANGLEAXIS_HPP_
#define ANGLEAXIS_HPP_

#include "../SO3.hpp"
#include "../../functional/so3/AngleAxis.hpp"

namespace kindr {
namespace core {
namespace oo {
namespace so3 {


template<typename Scalar_>
class AngleAxis : public SO3< AngleAxis<Scalar_> > {
 public:
  typedef SO3< AngleAxis<Scalar_> > Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  using Base::Base;
  using Base::operator =;

  double getAngle(){
    return this->getStorage().angle;
  }
  Vector3<Scalar> getAxis(){
    return this->getStorage().axis;
  }
};

} // namespace so3

template<typename Scalar_>
struct GetParam< so3::AngleAxis<Scalar_> > {
  typedef functional::so3::AngleAxis<Scalar_> type;
};


typedef so3::AngleAxis<float> AngleAxisF;
typedef so3::AngleAxis<double> AngleAxisD;

} // namespace oo
} // namespace core
} // namespace kindr

#endif /* ANGLEAXIS_HPP_ */
