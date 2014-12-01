#ifndef FUNCTIONAL_SE3_MATRIX4_HPP_
#define FUNCTIONAL_SE3_MATRIX4_HPP_

#include "../../LinAlg.hpp"

namespace kindr {
namespace core {
namespace functional {
namespace se3 {
template <typename Scalar_>
struct SE3Mat4 {
  typedef Scalar_ Scalar;
  typedef core::Matrix4<Scalar> Storage;
  typedef Vector3<Scalar> LieAlgebraVector;

  static Vector3<Scalar> apply(const Storage & se3, const Vector3<Scalar> & vec){
    return se3.template block<3, 3>(0, 0) * vec + se3.template block<3,1>(0, 3);
  }

  static Storage exp(const LieAlgebraVector & vec) {
    return Storage::Identity(); // TODO implement
  }

  static Storage compose(const Storage & a, const Storage & b){
    return a * b;
  }
};

} // namespace se3

typedef se3::SE3Mat4<float> SE3Mat4F;
typedef se3::SE3Mat4<double> SE3Mat4D;

} // namespace functional
} // namespace core
} // namespace kindr



#endif /* FUNCTIONAL_SE3_MATRIX4_HPP_ */
