Kindr - Kinematics and Dynamics for Robotics
-----------------------------------------------------------------
Autonomous Systems Lab
ETH Zurich

Author(s): Christian Gehring, Remo Diethelm, Hannes Sommer,
           Paul Furgale, Peter Fankhauser, Michael Bloesch, 
email    : leggedrobotics@ethz.ch
Date     : 08-Aug-2013

DOCUMENTATION
-----------------------------------------------------------------
Online: http://ethz-asl-lr.bitbucket.org/kindr
See also section 'Building the documentation' below.

REQUIREMENTS
-----------------------------------------------------------------
Linux
-----------------------------
GCC 4.7 is required at the minimum.

INSTALLATION
-----------------------------------------------------------------
Building the library:
-----------------------------
Build the library with CMake:
mkdir build
cd build
cmake ..
make

Building the documentation:
-----------------------------
Doxygen needs to be installed to create the documentation.

Build the documentation with doxygen:
mkdir build
cd build
cmake ..
make doc

The doxygen documentation can be found here:
rm_doc/doc/html/index.html


Building examples
-----------------------------
mkdir build
cd build
cmake ..
make demo

Building google tests
-----------------------------
GTests are built as soon as the folder gtest exists in the root folder.

Download and use GTest:
wget http://googletest.googlecode.com/files/gtest-1.7.0.zip
unzip gtest-1.7.0.zip
ln -s gtest-1.7.0 gtest
mkdir build
cd build
cmake ..
make
