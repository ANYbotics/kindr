Kindr - Kinematics and Dynamics for Robotics
-----------------------------------------------------------------
Autonomous Systems Lab
ETH Zurich

Contact  : Christian Gehring [gehrinch ( at ) ethz.ch]

Author(s): Michael Bloesch, Remo Diethelm, Peter Fankhauser, Paul Furgale, Christian Gehring, Hannes Sommer

Date     : 08-Aug-2013

DOCUMENTATION
-----------------------------------------------------------------
[Online](http://ethz-asl-lr.bitbucket.org/kindr)

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
```bash
kindr$ mkdir build
kindr$ cd build
kindr$ cmake ..
kindr$ make
```

Building the documentation:
-----------------------------
Doxygen needs to be installed to create the documentation.

Build the documentation with doxygen:
```bash
kindr$ mkdir build
kindr$ cd build
kindr$ cmake ..
kindr$ make doc
```

The doxygen documentation can be found here:

kindr$ doc/doxygen/doc/html/index.html


Building google tests
-----------------------------
GTests are built as soon as the folder gtest exists in the root folder.

Download and use GTest:

```bash
kindr$ wget http://googletest.googlecode.com/files/gtest-1.7.0.zip
kindr$ unzip gtest-1.7.0.zip
kindr$ ln -s gtest-1.7.0 gtest
kindr$ mkdir build
kindr$ cd build
kindr$ cmake ..
kindr$ make
```
