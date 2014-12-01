#ifndef FUNCTIONAL_ANGLEAXIS_HPP_
#define FUNCTIONAL_ANGLEAXIS_HPP_

#include <Eigen/Geometry>

#include "../../LinAlg.hpp"

namespace kindr {
namespace core {
namespace functional {
namespace so3 {

template<typename Scalar_>
struct AngleAxis {
  typedef Scalar_ Scalar;
  typedef Vector3<Scalar> LieAlgebraVector;
  struct Storage{
    Scalar angle;
    Vector3<Scalar> axis;
    bool operator == (const Storage& other) const { return angle == other.angle && axis == other.axis;};
  };

  static Vector3<Scalar> apply(const Storage & angleAxis, const Vector3<Scalar> & v){
    return Eigen::AngleAxis<Scalar>(angleAxis.angle, angleAxis.axis)*v;
  }

  static Storage invert(const Storage & angleAxis){
    return {angleAxis.angle, -angleAxis.axis};
  }

  static Storage exp(const LieAlgebraVector & v){
    //TODO delegate to correct stuff
    const double n = v.norm();
    if(n < 1e-9) return { 0, Vector3<Scalar>::Zero().eval() };
    return {n, v / n};
  }

 private:
  AngleAxis() = delete;
};

} // namespace so3

typedef so3::AngleAxis<float> AngleAxisF;
typedef so3::AngleAxis<double> AngleAxisD;

} // namespace functional
} // namespace core
} // namespace kindr


#endif /* FUNCTIONAL_ANGLEAXIS_HPP_ */
