Kindr - Kinematics and Dynamics for Robotics
=============================================

**Kindr 1.0.0 released!** (28.06.2016, see changelog below)

Autonomous Systems Lab
ETH Zurich

Contact  : Christian Gehring [gehrinch ( at ) ethz.ch]

Author(s): C. Dario Bellicoso, Michael Bloesch, Remo Diethelm, Peter Fankhauser, Paul Furgale, Christian Gehring, Hannes Sommer

Date     : June-2016

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=kindr)](http://rsl-ci.ethz.ch/job/kindr/)

## Documentation

[Online](http://docs.leggedrobotics.com/kindr/)

Impatient individuals can directly download the [cheat sheet](http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf).

See also section 'Building the documentation' below.

## Changelog

### Kindr 1.0.0 

* Simplified header include `#include <kindr/Core>` is provided.
* Kindr is now strongly dependent on Eigen.
* All sub namespaces have been removed. (e.g. `kindr::rotations::eigen_impl` -> `kindr::`) 
* The implementations of rotations and time derivatives have been simplified (Passive, Hamiltonian).
    - Active typedefs (e.g. RotationQuaternionAD) have been removed and simpler ones (e.g. RotationQuaternionD) have been introduced.
    - Note that the functionality of some operators changed! Please check the [cheat sheet](http://ethz-asl-lr.bitbucket.org/kindr/cheatsheet_latest.pdf) to understand what is implemented.
    - Some hints on what needs to be changed from kindr 0.0.1:
       - `rotation.setFromVectors(v1, v2)` -> `rotation.setFromVectors(v2, v1)`
       - `C_BI.boxPlus(dt * B_w_IB)` -> `C_BI.boxPlus(dt * C_IB * B_w_IB)`
       - `C_BI.boxMinus(dt *  B_w_IB)` ->  `-C_BI.boxMinus(dt * B_w_IB)`
       - Euler angles probably have to be negated.
* Conversion methods between ROS and kindr have been moved to the package [kindr_ros](https://github.com/ethz-asl/kindr_ros).
* Concatenation of Homogeneous Transformation is now implemented. 
* Short typedefs are provided for Homogeneous Transformation: `HomTransformQuatD`, `HomTransformMatrixD`.
* Jacobian of exponential map is implemented.
* Unit tests based on gtest are provided to test the convention of other software packages.
    - Gazebo (gazebo::math::Quaternion) uses the same convention as kindr.
    - ROS TF (tf::Quaternion and tf::Matrix3x3) uses the same convention as kindr.
    - RBDL's RigidBodyDynamics::Math::SpatialTransform uses the same convention as kindr, whereas RBDL's RigidBodyDynamics::Math::Quaternion concatenates differently and its conversion to a rotation matrix is inverted.


## Requirements

* [Eigen 3.2.0](http://eigen.tuxfamily.org) (Older versions might also work)
* GCC 4.7 is required at the minimum.
* CMake 2.8.3 is required at the minimum.

## Installation

### Installing from packages (recommended for Ubuntu LTS users)

The maintainers of this project provide binary packages for ROS and Ubuntu
LTS releases. To install these packages, you may follow these instructions:

* Add the project PPA to your APT sources by issuing 

  ```
  sudo add-apt-repository ppa:ethz-asl/common
  ```

  on the command line

* To re-synchronize your package index files, run

  ```
  sudo apt-get update
  ```

* Install all project packages and their dependencies through

  ```
  sudo apt-get install ros-indigo-kindr-*
  ```

  or selected packages using your favorite package management tool.

### Building with cmake

Install the library with [CMake](www.cmake.org):

```bash
mkdir build
cd build
cmake ..
sudo make install
```

Uninstall the library with:

```bash
cd build
sudo sudo make uninstall
```

Kindr can be included in your cmake project.
Add the following to your *CmakeLists.txt*:

```
find_package(kindr) 
include_directories(${kindr_INCLUDE_DIRS}) 
```

### Building with catkin

Build kindr with [catkin](wiki.ros.org/catkin):

```bash
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/kindr.git
catkin_make_isolated -C ~/catkin_ws
```

or with [catkin command line tools](http://catkin-tools.readthedocs.org):

```bash
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/kindr.git
catkin build -w ~/catkin_ws kindr
```

Kindr can be included in your catkin project with:
Add the following to your *CMakeLists.txt*:
```
find_package(catkin COMPONENTS kindr) 
include_directories(${catkin_INCLUDE_DIRS}) 
```

And to your *package.xml*:

```xml
<package>
	<build_depend>kindr</build_depend>
</package>
```


### Building the documentation

Build the documentation with [Doxygen](www.doxygen.org):
```bash
mkdir build
cd build
cmake ..
make doc
```

The doxygen documentation can be found here:

```
doc/doxygen/doc/html/index.html
```

### Building unit tests with gtest

[GTests](https://code.google.com/p/googletest/) are only built if the folder *gtest* exists in the root folder.

Download and use GTest:

```bash
wget http://googletest.googlecode.com/files/gtest-1.7.0.zip
unzip gtest-1.7.0.zip
ln -s gtest-1.7.0 gtest
mkdir build
cd build
cmake  .. -DBUILD_TEST=true
make
```
