#ifndef SE3_HPP_
#define SE3_HPP_

#include <type_traits>
#include "../LinAlg.hpp"

#include "../functional/Conversions.hpp"

namespace kindr {
namespace core {
namespace oo {

template<typename ParamType>
struct GetParam; // TODO find better name

template <typename Derived>
class SE3 {
 public:
  typedef typename GetParam<Derived>::type Parameterization;
  typedef typename Parameterization::Storage Storage;
  typedef typename Parameterization::Scalar Scalar;
  typedef typename Parameterization::LieAlgebraVector LieAlgebraVector;

  // Constructors:
  SE3() = default;

  SE3(const SE3 &) = default;
  SE3(SE3 &&) = default;

  template<typename Other>
  explicit SE3(const SE3<Other> & other){
    ConversionInto<Other>::convertInto(other.getStorage(), this->getStorage());
  }

  SE3(const Storage & storage) : storage_(storage) {
    static_assert(std::is_base_of<SE3, Derived>::value, "Derived must be a child of SE3");
    static_assert(sizeof(SE3) == sizeof(Derived), "Derived must not have any data except through SE3's storage");
  }

  // Assignment operators:
  Derived & operator = (const SE3 & other){ getStorage() = other.getStorage(); return *this; }
  template <typename Other>
  Derived & operator = (const SE3<Other> & other){ ConversionInto<Other>::convertInto(other.getStorage(), this->getStorage()); return *this; }

  // SE3 interface:
  Vector3<Scalar> apply(const Vector3<Scalar> & vec) const { return Parameterization::apply(storage_, vec); }
  template <int Colums>
  Matrix3N<Scalar, Colums> applyColumnwise(const Matrix3N<Scalar, Colums> & columVectors) const { return Parameterization::apply(storage_, columVectors); }

  Derived inverse() const { return Parameterization::inverse(storage_); }

  inline Derived operator * (const Derived & other) const { return Parameterization::compose(getStorage(), other.getStorage()); }

  inline bool operator == (const Derived & other) const { return getStorage() == other.getStorage(); }

  static Derived exp(const LieAlgebraVector & vec) { return Parameterization::exp(vec); }

  // Homogeneous coordinates interface //TODO better name or overload solution!
  Vector4<Scalar> applyToHom(const Vector4<Scalar> & homCoords) const { return Parameterization::apply(storage_, homCoords); }
  template <int Cols>
  Matrix4N<Scalar, Cols> applyToHomColumnwise(const Matrix4N<Scalar, Cols> & colmnwiseHomCoords) const { return Parameterization::apply(storage_, colmnwiseHomCoords); }


  // upcasts to derived Derived
  operator Derived & () { static_cast<Derived &>(*this); }
  operator Derived && () & { static_cast<Derived &&>(*this); }
  operator const Derived & () const { static_cast<const Derived &>(*this); }

  // storage accessors:
  Storage& getStorage() { return storage_; }
  const Storage& getStorage() const { return storage_; }

  // applying operators:
  Vector3<Scalar> operator ()(const Vector3<Scalar> & vec) const { return apply(vec); }
  Vector3<Scalar> operator * (const Vector3<Scalar> & vec) const { return apply(vec); }


  // cast method
  template <typename Dest>
  Dest cast() const {
    return Dest(*this);
  }

 private:
  template <typename Other>
  using ConversionInto = functional::Conversions<typename Other::Parameterization, Parameterization>;

  Storage storage_;
};


} // namespace oo
} // namespace core
} // namespace kindr

#endif /* SE3_HPP_ */
