#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_


#include <type_traits>

#include <Eigen/Geometry>


namespace kindr {
namespace core {
namespace functional {

template<typename From, typename To>
struct Conversions {
  static void convertInto(const From & /*from*/, To & /*into*/){
    static_assert(sizeof(From) < 0, "Conversion not supported");
  }
};

template<typename From, typename To>
inline void convertFromInto(const typename From::Storage & from, typename To::Storage & into){
  Conversions<From, To>::convertInto(from, into);
}

template<typename From, typename To>
inline typename To::Storage convertFromTo(typename From::Storage & from){
  typename To::Storage into;
  convertFromInto<From, To>(from, into);
  return into;
}

template <typename ParamType>
struct Conversions< ParamType, ParamType > {
  static void convertInto(const ParamType & from, ParamType & into){
    into = from;
  }
};

namespace so3 {
  template<typename Scalar_>
  struct SO3Mat;
  template<typename Scalar_>
  struct AngleAxis;
}

//TODO support converting scalar
template <typename Scalar>
struct Conversions<so3::AngleAxis<Scalar>, so3::SO3Mat<Scalar> > {
  static void convertInto(const typename so3::AngleAxis<Scalar>::Storage & from, typename so3::SO3Mat<Scalar>::Storage & into){
    into = Eigen::AngleAxis<Scalar>(from.angle, from.axis);
  }
};
template <typename Scalar>
struct Conversions<so3::SO3Mat<Scalar>, so3::AngleAxis<Scalar> > {
  static void convertInto(const typename so3::SO3Mat<Scalar>::Storage & from, typename so3::AngleAxis<Scalar>::Storage & into){
    Eigen::AngleAxis<Scalar> aa(from);
    into.angle = aa.angle();
    into.axis = aa.axis();
  }
};

} // namespace functional
} // namespace core
} // namespace kindr

#endif /* CONVERSIONS_HPP_ */
