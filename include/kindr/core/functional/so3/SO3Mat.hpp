#ifndef KINDR_CORE_FUNCTIONAL_SO3MAT_HPP_
#define KINDR_CORE_FUNCTIONAL_SO3MAT_HPP_

#include "../../LinAlg.hpp"

namespace kindr {
namespace core {
namespace functional {
namespace so3 {

template<typename Scalar_>
struct SO3Mat {
  typedef Scalar_ Scalar;
  typedef Matrix3<Scalar> Storage;
  typedef Vector3<Scalar> LieAlgebraVector;

  static Vector3<Scalar> apply(const Storage & M, const Vector3<Scalar> & v){
    return M*v;
  }

  static Storage invert(const Storage & M){
    return M.transpose();
  }

 private:
  SO3Mat() = delete;
};

} // namespace so3

typedef so3::SO3Mat<float> SO3MatF;
typedef so3::SO3Mat<double> SO3MatD;

} // namespace functional
} // namespace core
} // namespace kindr

#endif /* KINDR_CORE_FUNCTIONAL_SO3MAT_HPP_ */
