#ifndef VECTOR_HPP_
#define VECTOR_HPP_

#include "../T3.hpp"
#include "../../functional/t3/T3Vec.hpp"

namespace kindr {
namespace core {
namespace oo {
namespace t3 {


template<typename Scalar_>
struct T3Vec : public T3< T3Vec<Scalar_> >{
  typedef T3< T3Vec<Scalar_> > Base;
  using Base::Base;
  using Base::operator =;

  double & x() {
    return this->getStorage()[0];
  };
  double & y() {
    return this->getStorage()[1];
  };
  double & z() {
    return this->getStorage()[2];
  };
  //TODO implement const versions
};

} // namespace t3

template<typename Scalar_>
struct GetParam< t3::T3Vec<Scalar_> > {
  typedef functional::t3::T3Vec<Scalar_> type;
};

typedef t3::T3Vec<float> T3VecF;
typedef t3::T3Vec<double> T3VecD;

} // namespace oo
} // namespace core
} // namespace kindr

#endif /* VECTOR_HPP_ */
