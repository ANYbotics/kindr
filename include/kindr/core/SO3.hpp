/*
 * SO3.hpp
 *
 *  Created on: Nov 17, 2014
 *      Author: hannes
 */

#ifndef INCLUDE_KINDR_CORE_SO3_HPP_
#define INCLUDE_KINDR_CORE_SO3_HPP_

#include "SE3.hpp"

namespace kindr {
namespace core {

template<class Derived>
class SO3 : public SE3<Derived> {
 public:
  typedef SE3< Derived > Base;
  using Base::Base;
};

}
}

//TODO move to separate files

#include <Eigen/Geometry>

namespace kindr {
namespace core {


template<typename Scalar_>
class SO3Mat : public SO3< SO3Mat<Scalar_> > {
 public:
  typedef SO3< SO3Mat<Scalar_> > Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  using Base::Base;

  Storage & getMatrix(){
    return this->getStorage();
  }
};

template<typename Scalar_>
struct ParamTraits< SO3Mat<Scalar_> > {
  typedef Scalar_ Scalar;
  typedef Matrix3<Scalar> Storage;
  typedef Vector3<Scalar> LieAlgebraVector;

  static Vector3<Scalar> apply(const Storage & M, const Vector3<Scalar> & v){
    return M*v;
  }

  static Storage invert(const Storage & M){
    return M.transpose();
  }
};
typedef SO3Mat<float> SO3MatF;
typedef SO3Mat<double> SO3MatD;


template<typename Scalar_>
class AngleAxis : public SO3< AngleAxis<Scalar_> > {
 public:
  typedef SO3< AngleAxis<Scalar_> > Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  using Base::Base;

  double getAngle(){
    return this->getStorage().angle;
  }
  Vector3<Scalar> getAxis(){
    return this->getStorage().axis;
  }
};
template<typename Scalar_>
struct ParamTraits< AngleAxis<Scalar_> > {
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
};

typedef AngleAxis<float> AngleAxisF;
typedef AngleAxis<double> AngleAxisD;


//TODO support converting scalar
template <typename Scalar>
struct CastTraits< AngleAxis<Scalar>, SO3Mat<Scalar> > {
  static void convertInto(const AngleAxis<Scalar> & from, SO3Mat<Scalar> & into){
    into.getStorage() = Eigen::AngleAxis<Scalar>(from.getStorage().angle, from.getStorage().axis);
  }
};
template <typename Scalar>
struct CastTraits< SO3Mat<Scalar>, AngleAxis<Scalar> > {
  static void convertInto(const SO3Mat<Scalar> & from, AngleAxis<Scalar> & into){
    Eigen::AngleAxis<Scalar> aa(from.getStorage());
    into.getStorage().angle = aa.angle();
    into.getStorage().axis = aa.axis();
  }
};


}
}


#endif /* INCLUDE_KINDR_CORE_SO3_HPP_ */
