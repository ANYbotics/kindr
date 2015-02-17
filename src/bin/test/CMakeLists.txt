remake_find_package(Threads REQUIRED)

remake_add_test(
  GOOGLE skew_matrix_from_vector_gtest
  test_main.cpp linear_algebra/SkewMatrixFromVectorTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)

remake_add_test(
  GOOGLE force_gtest
  test_main.cpp phys_quant/ForceTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)

remake_add_test(
  GOOGLE pose_eigen_gtest
  test_main.cpp poses/PoseEigentTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_test(
  GOOGLE pose_diff_eigen_gtest
  test_main.cpp poses/PoseDiffEigentTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)

remake_add_test(
  GOOGLE position_eigen_gtest
  test_main.cpp positions/PositionEigentTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_test(
  GOOGLE position_diff_eigen_gtest
  test_main.cpp positions/PositionDiffEigentTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)

remake_add_test(
  GOOGLE quaternion_eigen_gtest
  test_main.cpp quaternions/QuaternionEigenTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)

remake_add_test(
  GOOGLE rotation_eigen_gtest
  test_main.cpp rotations/RotationEigenTest.cpp
    rotations/eigen/RotationMatrixTest.cpp
    rotations/eigen/RotationQuaternionTest.cpp
    rotations/eigen/RotationVectorTest.cpp
    rotations/eigen/AngleAxisTest.cpp
    rotations/eigen/RotationMatrixTest.cpp
    rotations/eigen/EulerAnglesZyxTest.cpp
    rotations/eigen/EulerAnglesXyzTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_test(
  GOOGLE rotation_diff_eigen_gtest
  test_main.cpp rotations/RotationDiffEigenTest.cpp
    rotations/eigen/LocalAngularVelocityTest.cpp
    rotations/eigen/RotationQuaternionDiffTest.cpp
    rotations/eigen/RotationMatrixDiffTest.cpp
    rotations/eigen/EulerAnglesZyxDiffTest.cpp
    rotations/eigen/EulerAnglesXyzDiffTest.cpp  
  LINK ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_test(
  GOOGLE rotation_quaternion_gtest
  test_main.cpp rotations/eigen/RotationQuaternionTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)

remake_add_test(
  GOOGLE vectors_gtest
  test_main.cpp vectors/VectorsTest.cpp
  LINK ${CMAKE_THREAD_LIBS_INIT}
)
