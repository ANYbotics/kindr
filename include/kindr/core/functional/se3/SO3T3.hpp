#ifndef FUNCTIONAL_SE3_SO3T3_HPP_
#define FUNCTIONAL_SE3_SO3T3_HPP_

#include <type_traits>

#include "../../LinAlg.hpp"
#include "../t3/T3Vec.hpp"

namespace kindr {
namespace core {
namespace functional {
namespace se3 {

template <typename SO3ParamType_>
using DefaultT3 = t3::T3Vec<typename SO3ParamType_::Scalar>;

template<typename SO3ParamType_, typename T3ParamType_ = DefaultT3<SO3ParamType_>>
struct SO3T3Pair {
  typedef SO3ParamType_ SO3ParamType;
  typedef T3ParamType_ T3ParamType;
  typedef typename SO3ParamType::Scalar Scalar;

  typedef Vector6<Scalar> LieAlgebraVector;
  struct Storage {
    typename SO3ParamType::Storage so3;
    typename T3ParamType::Storage t3;
    bool operator == (const Storage& other) const {
      return t3 == other.t3 && so3 == other.so3;
    }
  };

 protected:
  SO3T3Pair() {
    static_assert(std::is_same<Scalar, typename T3ParamType::Scalar>::value,
                  "Pairs of SO3 and T3 with different scalar are not supported.");
  }
};

template<typename SO3ParamType_, typename T3ParamType_ = DefaultT3<SO3ParamType_>>
struct SO3AfterT3 : public SO3T3Pair<SO3ParamType_, T3ParamType_> {
  typedef SO3T3Pair<SO3ParamType_, T3ParamType_> Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  static Vector3<Scalar> apply(const Storage & t, const Vector3<Scalar> & vec) {
    return SO3ParamType_::apply(t.so3, T3ParamType_::apply(t.t3, vec));
  }
};

template<typename SO3ParamType_, typename T3ParamType_ = DefaultT3<SO3ParamType_>>
struct T3AfterSO3 : public SO3T3Pair<SO3ParamType_, T3ParamType_> {
  typedef SO3T3Pair<SO3ParamType_, T3ParamType_> Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  static Vector3<Scalar> apply(const Storage & t, const Vector3<Scalar> & vec) {
    return t.t3.apply(t.so3.apply(vec));
  }
};

} // namespace se3
} // namespace functional
} // namespace core
} // namespace kindr


#endif /* FUNCTIONAL_SE3_SO3T3_HPP_ */
