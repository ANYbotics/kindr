remake_find_package(eigen3 CONFIG)

remake_include(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

remake_add_directories(include bin)
