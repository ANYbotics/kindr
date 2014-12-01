#ifndef SO3T3_HPP_
#define SO3T3_HPP_

#include "../SE3.hpp"
#include "../t3/T3Vec.hpp"
#include "../../functional/se3/SO3T3.hpp"

namespace kindr {
namespace core {
namespace oo {
namespace se3 {

template <typename SO3ParamType_>
using DefaultT3 = t3::T3Vec<typename SO3ParamType_::Scalar>;

template <typename SO3ParamType_, typename T3ParamType_ = DefaultT3<SO3ParamType_> >
class SO3AfterT3: public SE3<SO3AfterT3<SO3ParamType_, T3ParamType_>> {
 public:
  typedef SO3ParamType_ SO3ParamType;
  typedef T3ParamType_ T3ParamType;
  typedef SE3<SO3AfterT3<SO3ParamType_, T3ParamType_> > Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  using Base::Base; // inherit constructors
  SO3AfterT3(const SO3ParamType & so3, const T3<Scalar> & t3) : Base(Storage({so3, t3})) {}

  SO3ParamType & getSo3() { return this->getStorage().so3; }
  const SO3ParamType & getSo3() const { return this->getStorage().so3; }
  T3<Scalar> & getT3() { return this->getStorage().t3; }
  const T3<Scalar> & getT3() const { return this->getStorage().t3; }
};

template <typename SO3ParamType_, typename T3ParamType_ = DefaultT3<SO3ParamType_> >
class T3AfterSO3: public SO3AfterT3<SO3ParamType_, T3ParamType_> {
 public:
  typedef SO3ParamType_ SO3ParamType;
  typedef T3ParamType_ T3ParamType;
  typedef SE3<SO3AfterT3<SO3ParamType_, T3ParamType_> > Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  using Base::Base; // inherit constructors
  T3AfterSO3(const SO3ParamType & so3, const T3<Scalar> & t3) : Base(Storage({so3, t3})) {}

  SO3ParamType & getSo3() { return this->getStorage().so3; }
  const SO3ParamType & getSo3() const { return this->getStorage().so3; }
  T3<Scalar> & getT3() { return this->getStorage().t3; }
  const T3<Scalar> & getT3() const { return this->getStorage().t3; }
};
} // namespace se3

template<typename SO3ParamType_, typename T3ParamType_>
struct GetParam< se3::SO3AfterT3<SO3ParamType_, T3ParamType_> > {
  typedef functional::se3::SO3AfterT3<typename SO3ParamType_::Storage, typename T3ParamType_::Storage> type;
};

template<typename SO3ParamType_, typename T3ParamType_>
struct GetParam< se3::T3AfterSO3<SO3ParamType_, T3ParamType_> > {
  typedef functional::se3::SO3AfterT3<typename SO3ParamType_::Storage, typename T3ParamType_::Storage> type;
};


template <typename SO3ParamType, typename T3ParamType>
se3::SO3AfterT3<SO3ParamType, T3ParamType> operator * (const SO3<SO3ParamType> & so3, const T3<T3ParamType> & t3){
  return se3::SO3AfterT3<SO3ParamType, T3ParamType>(so3, t3);
}

template <typename SO3ParamType, typename T3ParamType>
se3::SO3AfterT3<SO3ParamType, T3ParamType> operator * (const T3<T3ParamType> & t3, const SO3<SO3ParamType> & so3){
  return se3::T3AfterSO3<SO3ParamType, T3ParamType>(so3, t3);
}

} // namespace oo
} // namespace core
} // namespace kindr

#endif /* SO3T3_HPP_ */
