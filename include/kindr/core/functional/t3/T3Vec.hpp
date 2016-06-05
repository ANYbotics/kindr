#ifndef PVECTOR_HPP_
#define PVECTOR_HPP_

#include "../../LinAlg.hpp"

namespace kindr {
namespace core {
namespace functional {
namespace t3 {

template<typename Scalar_>
struct T3Vec {
  typedef Scalar_ Scalar;
  typedef Vector3<Scalar> Storage;
  typedef Vector3<Scalar> LieAlgebraVector;

  static Vector3<Scalar> apply(const Storage & t, const Vector3<Scalar> & vec){
    return t + vec;
  }
  static Storage exp(const LieAlgebraVector & vec){
    return vec;
  }
};

} // namespace t3
typedef t3::T3Vec<float> T3VecF;
typedef t3::T3Vec<double> T3VecD;
} // namespace functional
} // namespace core
} // namespace kindr

#endif /* PVECTOR_HPP_ */
