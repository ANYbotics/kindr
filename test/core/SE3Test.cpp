/*
 * testSE3.cpp
 *
 *  Created on: Nov 14, 2014
 *      Author: hannes
 */


#include <cmath>
#include <gtest/gtest.h>
#include <kindr/core/SE3.hpp>
#include <kindr/core/T3.hpp>
#include <kindr/core/SO3.hpp>

using namespace kindr::core;
const Vector3d zero = Vector3d::Zero();
const Vector3d x = Vector3d::UnitX();
const Vector3d y = Vector3d::UnitY();
const Vector3d z = Vector3d::UnitZ();
const Matrix4d MatId4x4 = Matrix4d::Identity();

const double halfPi = M_PI / 2.0;


TEST(SE3Test, testCompileAndShowcase) {
  // construct
  SE3Matrix4D se3Id = SE3Matrix4D::exp(zero);
  SE3Matrix4D se3Id2(MatId4x4);

  Translation3D transX(x);
  AngleAxisD angleAxisHalfPiX({halfPi, x});
  AngleAxisD angleAxisOneX = AngleAxisD::exp(x);

  // conversion variants
  SO3MatD R1 = angleAxisHalfPiX.cast<SO3MatD>(); // cast method
  SO3MatD R2(angleAxisHalfPiX); // constructor
  SO3MatD R3; R3 = angleAxisHalfPiX; // assignment

  // access data
  EXPECT_EQ(1, angleAxisOneX.getAngle());
  EXPECT_EQ(1, angleAxisOneX.getStorage().angle);
  EXPECT_EQ(x, angleAxisOneX.getAxis());
  EXPECT_EQ(x, angleAxisOneX.getStorage().axis);
  EXPECT_EQ(MatId4x4, se3Id.getMatrix4());
  EXPECT_EQ(MatId4x4, se3Id2.getMatrix4());
  EXPECT_EQ(MatId4x4, se3Id2.getStorage());

  Matrix3d matHalfPiX(Eigen::AngleAxis<double>(halfPi, x));
  EXPECT_NEAR((matHalfPiX - R1.getMatrix()).norm(), 0, 1e-9);
  EXPECT_NEAR((matHalfPiX - R2.getMatrix()).norm(), 0, 1e-9);
  EXPECT_NEAR((matHalfPiX - R3.getMatrix()).norm(), 0, 1e-9);

  // apply se3Id:
  EXPECT_EQ(x, se3Id.apply(x));
  EXPECT_EQ(x, se3Id(x));
  EXPECT_EQ(x, se3Id * x);
  EXPECT_EQ(x, angleAxisOneX(x));
  EXPECT_EQ(x + y, transX(y));
  EXPECT_NEAR((z - angleAxisHalfPiX(y)).norm(), 0, 1e-9); //TODO use proper Eigen gtest stuff. ethz-asl/eigen_checks?

  // compose
  EXPECT_EQ(MatId4x4, (se3Id * se3Id).getMatrix4());
  EXPECT_EQ(SO3AfterT3<AngleAxisD>(angleAxisHalfPiX, transX), angleAxisHalfPiX * transX); // an experimental support for composition across different SE3 types
  EXPECT_NEAR((x + z - (angleAxisHalfPiX * transX).apply(y)).norm(), 0, 1e-9);
}

// ***************** High level stuff : involving the concept of frames

// ** factory concept: simple, but not type safe
namespace factories {
// TODO find a better naming scheme
  template <typename Scalar>
  static AngleAxis<Scalar> rotateVectorAroundAxis( Scalar angle, Vector3<Scalar> axis ) {
    return AngleAxis<Scalar>({angle, axis});
  }
  template <typename SO3ParamType>
  static typename AngleAxis<typename SO3ParamType::Scalar>::Storage asRotateVectorAroundAxis( const SO3<SO3ParamType> & so3 ) {
    return so3.template cast< AngleAxis<typename SO3ParamType::Scalar> >().getStorage();
  }
  template <typename Scalar>
  static AngleAxis<Scalar> rotateFrameAroundAxis( Scalar angle, Vector3<Scalar> axis ) {
    return AngleAxis<Scalar>({angle, axis}).invert();
  }
  template <typename SO3ParamType>
  static typename AngleAxis<typename SO3ParamType::Scalar>::Storage asRotateFrameAroundAxis( const SO3<SO3ParamType> & so3 ) {
    return so3.template cast< AngleAxis<typename SO3ParamType::Scalar> >().invert().getStorage();
  }
}

TEST(SE3Test, modelLevelExperiments_factoryBased) {
  SO3MatD mRotateVector(factories::rotateVectorAroundAxis( 1.0, x ));
  SO3MatD mRotateFrame(factories::rotateFrameAroundAxis( 1.0, x ));

  EXPECT_NEAR((mRotateVector.invert().getMatrix() - mRotateFrame.getMatrix()).norm(), 0, 1e-9);

  EXPECT_EQ(x , factories::asRotateVectorAroundAxis( mRotateVector ).axis);
  EXPECT_EQ(x , factories::asRotateFrameAroundAxis( mRotateFrame ).axis);

  //BUT: this is one of the drawbacks of this factory based method
  EXPECT_EQ(-x , factories::rotateFrameAroundAxis( 1.0, x ).getAxis());
}


// ** frame based typing : more verbose to use but type safe
template <typename Scalar, typename Frame>
class VectorInFrame : public Vector3<Scalar> {
 public:
  explicit VectorInFrame(const  Vector3<Scalar> & v) : Vector3<Scalar>(v) {}
  VectorInFrame() = default;
  VectorInFrame(VectorInFrame &&) = default;
  VectorInFrame(const VectorInFrame &) = default;
  VectorInFrame & operator = (const VectorInFrame &) = default;
  VectorInFrame & operator = (VectorInFrame &&) = default;
};

template <typename ToFrame, typename FromFrame, typename Param>
class Transformation;

template <typename Frame, typename Param>
class ActiveSE3 {
 public:
  explicit ActiveSE3(const SE3<Param> & se3) : se3_(se3) {}
  explicit ActiveSE3(const typename SE3<Param>::Storage & se3) : se3_(se3) {}

  typedef VectorInFrame<typename Param::Scalar, Frame> Vector;

  Vector apply(const Vector & v) const { return Vector(se3_.apply(v)); }

  ActiveSE3 invert() const {
    return ActiveSE3(se3_.invert());
  }

  template <typename OtherParam>
  ActiveSE3 operator * (const ActiveSE3<Frame, OtherParam> & other) const{
    return ActiveSE3(se3_ * other.se3_);
  }


  template <typename RotatedFrame>
  const Transformation<Frame, RotatedFrame, Param> & asTransformationFromRotatedFrameToThisFrame() const;

 private:
  template <typename OToFrame, typename OFromFrame, typename OParam> friend class Transformation;
  SE3<Param> se3_;
};

template <typename Frame>
using ActiveAngleAxisD = ActiveSE3<Frame, AngleAxisD>;

template <typename ToFrame, typename FromFrame, typename Param>
class Transformation {
 public :
  explicit Transformation(const SE3<Param> & se3) : se3_(se3) {}
  explicit Transformation(const typename SE3<Param>::Storage & se3) : se3_(se3) {}
  typedef VectorInFrame<typename Param::Scalar, ToFrame> ToVector;
  typedef VectorInFrame<typename Param::Scalar, FromFrame> FromVector;

  ToVector apply(const FromVector & v) { return ToVector(se3_.apply(v)); }

  Transformation<FromFrame, ToFrame, Param> invert(){
    return Transformation<FromFrame, ToFrame, Param>(se3_.invert());
  }
  template <typename OtherFromFrame, typename OtherParam>
  Transformation<ToFrame, OtherFromFrame, Param> operator * (const Transformation<FromFrame, OtherFromFrame, OtherParam> & other){
    return Transformation<ToFrame, OtherFromFrame, Param>(se3_ * other.se3_);
  }

  const ActiveSE3<ToFrame, Param> & asActiveInToFrame() {
    return reinterpret_cast<ActiveSE3<ToFrame, Param> &>(*this);
  }
  ActiveSE3<FromFrame, Param> asActiveInFromFrame() {
    return ActiveSE3<FromFrame, Param>(se3_.invert());
  }
 private:
  template <typename OToFrame, typename OFromFrame, typename OParam> friend class Transformation;
  SE3<Param> se3_;
};

template <typename Frame, typename Param>
template <typename RotatedFrame>
const Transformation<Frame, RotatedFrame, Param> & ActiveSE3<Frame, Param>::asTransformationFromRotatedFrameToThisFrame() const {
  return reinterpret_cast<const Transformation<Frame, RotatedFrame, Param> &>(*this);
}

template <typename FromFrame, typename ToFrame>
using ActiveAngleTransformationD = Transformation<FromFrame, ToFrame, AngleAxisD>;



struct FrameA{};
struct FrameB{};

TEST(SE3Test, modelLevelExperiments_modelBasedTyping) {
  typedef VectorInFrame<double, FrameA> VectorA;
  typedef VectorInFrame<double, FrameB> VectorB;

  ActiveAngleAxisD<FrameA> R_A({halfPi, y});
  VectorA x_A(x);
  VectorA xRotatedInA(R_A.apply(x_A));
  xRotatedInA = R_A.apply(x_A);

  VectorB z_B(z);
  // z_B =  R_A.apply(x_A); // would not compile!
  // x_A =  R_A.apply(z_B); // would not compile!

  ActiveAngleTransformationD<FrameA, FrameB>
    T_A_B = R_A.asTransformationFromRotatedFrameToThisFrame<FrameB>();

  VectorA zInBTransformedToA = T_A_B.apply(z_B);

  // T_A_B.apply(x_A); // would not compile!

  EXPECT_NEAR((zInBTransformedToA - x_A).norm(), 0, 1e-9);
}
