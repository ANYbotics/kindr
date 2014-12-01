#ifndef OO_SE3_MATRIX4_HPP_
#define OO_SE3_MATRIX4_HPP_

#include "../SE3.hpp"
#include "../../functional/se3/SE3Mat4.hpp"

namespace kindr {
namespace core {
namespace oo {
namespace se3 {

template <typename Scalar_>
class SE3Mat4: public SE3<SE3Mat4<Scalar_>> { // TODO find better name
 public:
  typedef SE3<SE3Mat4<Scalar_>> Base;
  typedef typename Base::Scalar Scalar;
  typedef typename Base::Storage Storage;

  using Base::Base;

  const Storage & getMatrix4() const {
    return this->getStorage();
  }
};

} // namespace se3

template <typename Scalar_>
struct GetParam<se3::SE3Mat4<Scalar_> > {
  typedef functional::se3::SE3Mat4<Scalar_> type;
};

typedef se3::SE3Mat4<float> SE3Mat4F;
typedef se3::SE3Mat4<double> SE3Mat4D;
} // namespace oo
} // namespace core
} // namespace kindr



#endif /* OO_SE3_MATRIX4_HPP_ */
