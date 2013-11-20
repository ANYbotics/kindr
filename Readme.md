Math library for Robotics
-----------------------------------------------------------------
Math library to extend common C++ libraries such as Eigen, Boost, 
and GNU Scientific Library.

-----------------------------------------------------------------
Autonomous Systems Lab
ETH Zurich

Author(s): Christian Gehring, Remo Diethelm, Hannes Sommer, Peter Fankhauser, Michael Bloesch, 
email    : leggedrobotics@ethz.ch
Date     : 08-Aug-2013


INSTALLATION
-----------------------------------------------------------------
Building the library:
-----------------------------
mkdir build
cd build
cmake ..
make

Building the documentation:
-----------------------------
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
wget http://googletest.googlecode.com/files/gtest-1.7.0.zip
unzip gtest-1.7.0.zip
ln -s gtest-1.7.0 gtest
mkdir build
cd build
cmake ..
make
