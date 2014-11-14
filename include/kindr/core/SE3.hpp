/*
 * SE3.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: hannes
 */

#ifndef SE3_HPP_
#define SE3_HPP_

#include "LinAlg.hpp"
#include <type_traits>

namespace kindr {
namespace core {

template<typename ParamType>
struct ParamTraits; // TODO find better name

template<typename From, typename To>
struct CastTraits;

template <typename Derived, typename ParamType = Derived>
class SE3 {
 public:
  typedef ParamTraits<ParamType> Parameterization;
  typedef typename Parameterization::Storage Storage;
  typedef typename Parameterization::Scalar Scalar;
  typedef typename Parameterization::LieAlgebraVector LieAlgebraVector;

  // Constructors:
  SE3() = default;

  SE3(const SE3 &) = default;
  SE3(SE3 &&) = default;

  template<typename Other>
  SE3(const SE3<Other> & other){
    CastTraits<Other, Derived>::convertInto(other, *this);
  }

  SE3(const Storage & storage) : storage_(storage) {
    static_assert(std::is_base_of<SE3, Derived>::value, "Derived must be a child of SE3");
    static_assert(sizeof(SE3) == sizeof(Derived), "Derived must not have any data except through SE3's storage");
  }

  // Assignment operators:
  Derived & operator = (const SE3 & other){ getStorage() = other.getStorage(); return *this; }
  template <typename Other>
  Derived & operator = (const SE3<Other> & other){ CastTraits<Other, Derived>::convertInto(other, *this); return *this; }

  // SE3 interface:
  Vector3<Scalar> apply(const Vector3<Scalar> & vec) const { return Parameterization::apply(storage_, vec); }
  template <int Colums>
  Matrix3N<Scalar, Colums> applyColumnwise(const Matrix3N<Scalar, Colums> & columVectors) const { return Parameterization::apply(storage_, columVectors); }

  Derived invert() { return Parameterization::invert(storage_); }

  inline Derived operator * (const Derived & other) const { return Parameterization::compose(getStorage(), other.getStorage()); }

  inline bool operator == (const Derived & other) const { return getStorage() == other.getStorage(); }

  static Derived exp(const LieAlgebraVector & vec) { return Parameterization::exp(vec); }

  // Homogeneous coordinates interface //TODO better name or overload solution!
  Vector4<Scalar> applyToHom(const Vector4<Scalar> & homCoords) const { return Parameterization::apply(storage_, homCoords); }
  template <int Colums>
  Matrix4N<Scalar, Colums> applyToHomColumnwise(const Matrix4N<Scalar, Colums> & columHomCoords) const { return Parameterization::apply(storage_, columHomCoords); }


  // upcasts to derived Derived
  operator Derived & () { static_cast<Derived &>(*this); }
  operator Derived && () & { static_cast<Derived &&>(*this); }
  operator const Derived & () const { static_cast<const Derived &>(*this); }

  // storage accessors:
  Storage& getStorage() { return storage_; }
  const Storage& getStorage() const { return storage_; }

  // delegating operators:
  Vector3<Scalar> operator ()(const Vector3<Scalar> & vec) const { return apply(vec); }
  Vector3<Scalar> operator * (const Vector3<Scalar> & vec) const { return apply(vec); }


  // cast method
  template <typename Dest>
  Dest cast() const {
    return Dest(*this);
  }
 private:
  Storage storage_;
};

template <typename ParamType>
struct CastTraits< ParamType, ParamType > {
  static void convertInto(const ParamType & from, ParamType & into){
    into = from;
  }
};

}
}

//TODO move to separate files

#include "T3.hpp"

namespace kindr {
namespace core {

template <typename SO3ParamType> // better name?
class SO3AfterT3: public SE3<SO3AfterT3<SO3ParamType>> { // better name? SO3T3?
 public:
  typedef SE3<SO3AfterT3<SO3ParamType>> Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  using Base::Base; // inherit constructors
  SO3AfterT3(const SO3ParamType & so3, const T3<Scalar> & t3) : Base(Storage({so3, t3})) {}

  SO3ParamType & getSo3() { return this->getStorage().so3; }
  const SO3ParamType & getSo3() const { return this->getStorage().so3; }
  T3<Scalar> & getT3() { return this->getStorage().t3; }
  const T3<Scalar> & getT3() const { return this->getStorage().t3; }
};

template<typename SO3ParamType_>
struct ParamTraits< SO3AfterT3<SO3ParamType_> > {
  typedef SO3ParamType_ SO3ParamType;
  typedef typename SO3ParamType::Scalar Scalar;

  typedef Vector3<Scalar> LieAlgebraVector;
  struct Storage {
    SO3ParamType so3;
    T3<Scalar> t3;
    bool operator == (const Storage& other) const { return t3 == other.t3 && so3 == other.so3 ;};
  };

  static Vector3<Scalar> apply(const Storage & t, const Vector3<Scalar> & vec) {
    return t.so3.apply(t.t3.apply(vec));
  }
};


template <typename SO3ParamType>
class SO3;

template <typename SO3ParamType>  // TODO make proper
SO3AfterT3<SO3ParamType> operator * (const SO3<SO3ParamType> & so3, const T3<typename SO3ParamType::Scalar> & t3){
  return SO3AfterT3<SO3ParamType>(SO3AfterT3<SO3ParamType>(so3, t3));
}

template <typename Scalar_>
class SE3Matrix4: public SE3<SE3Matrix4<Scalar_>> { // TODO find better name
 public:
  typedef SE3<SE3Matrix4<Scalar_>> Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  using Base::Base;

  const Storage & getMatrix4() const {
    return this->getStorage();
  }
};

template <typename Scalar_>
class ParamTraits<SE3Matrix4<Scalar_> > {
 public:
  typedef Scalar_ Scalar;
  typedef Matrix4<Scalar> Storage;
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

typedef SE3Matrix4<float> SE3Matrix4F;
typedef SE3Matrix4<double> SE3Matrix4D;

}
}

#endif /* SE3_HPP_ */
