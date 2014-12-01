#ifndef OO_SO3MAT_HPP_
#define OO_SO3MAT_HPP_

#include "../SO3.hpp"
#include "../../functional/so3/SO3Mat.hpp"

namespace kindr {
namespace core {
namespace oo {
namespace so3 {

template<typename Scalar_>
class SO3Mat : public SO3< SO3Mat<Scalar_> > {
 public:
  typedef SO3< SO3Mat<Scalar_> > Base;
  typedef typename Base::Storage Storage;

  using Base::Base;
  using Base::operator =;

  Storage & getMatrix(){
    return this->getStorage();
  }
};

} // namespace so3

template<typename Scalar_>
struct GetParam< so3::SO3Mat<Scalar_> > {
  typedef functional::so3::SO3Mat<Scalar_> type;
};

typedef so3::SO3Mat<float> SO3MatF;
typedef so3::SO3Mat<double> SO3MatD;
} // namespace oo
} // namespace core
} // namespace kindr


#endif /* OO_SO3MAT_HPP_ */
