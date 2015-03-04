Kindr - Kinematics and Dynamics for Robotics
============================================

Autonomous Systems Lab
ETH Zurich

Contact  : Christian Gehring [gehrinch ( at ) ethz.ch]

Author(s): Michael Bloesch, Remo Diethelm, Peter Fankhauser, Paul Furgale, Christian Gehring, Hannes Sommer

Date     : 08-Aug-2013

[![Build Status](http://129.132.38.183:8080/job/kindr/badge/icon)](http://129.132.38.183:8080/job/kindr/)

## Documentation

[Online](http://ethz-asl-lr.bitbucket.org/kindr)

Impatient individuals can directly download the [cheat sheet](http://ethz-asl-lr.bitbucket.org/kindr/cheatsheet_latest.pdf).

See also section 'Building the documentation' below.

## Requirements

### Linux
GCC 4.7 is required at the minimum.

## Installation

### Installing from packages (recommended for Ubuntu LTS users)

The maintainers of this project provide binary packages for the latest Ubuntu
LTS releases and commonly used system architectures. To install these packages,
you may follow these instructions:

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
  sudo apt-get install kindr-*
  ```

  or selected packages using your favorite package management tool

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
