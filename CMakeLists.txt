cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  Kindr
  VERSION 0.1
  RELEASE beta
  SUMMARY "Kinematics and Dynamics for Robotics."
  AUTHOR "Michael Bloesch"
  AUTHOR "Peter Fankhauser"
  AUTHOR "Paul Furgale"
  AUTHOR "Christian Gehring"
  AUTHOR "Hannes Sommer"
  CONTACT "gehrinch@ethz.ch"
  HOME "http://github.com/ethz-asl/kindr"
  LICENSE "BSD License (BSD)"
  README README.md
  PREFIX kindr-
  INSTALL /usr
  EXTRA_CXX_FLAGS "-std=c++0x"
)

remake_doc(
  html
)

remake_pack_deb(
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS kindr libeigen3-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libeigen3-dev libgtest-dev remake doxygen pkg-config
  PASS CMAKE_BUILD_TYPE KINDR_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libeigen3-dev libgtest-dev remake doxygen pkg-config
  PASS CMAKE_BUILD_TYPE KINDR_GIT_REVISION
)
