# Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
# Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Project configuration
cmake_minimum_required (VERSION 2.8)

project(kindr)

#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Noisily default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_definitions(-std=c++11)

# Don't build tests if not specified.
if(NOT BUILD_TEST)
  message(STATUS "Setting build-tests to false as not specified.")
  set(BUILD_TEST false CACHE BOOL "Choose whether to build tests." FORCE)
  set_property(CACHE BUILD_TEST PROPERTY STRINGS
    "True" "False")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gtest/" AND BUILD_TEST)
	message(STATUS "Building GTests!")
	option(BUILD_GTEST "build gtest" ON)
	add_subdirectory(gtest gtest)
	enable_testing()
endif()

# Add CMake module path
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Add common
#add_subdirectory(common)

# Add linear algebra 
#add_subdirectory(linear_algebra)

# Add quaternions
#add_subdirectory(quaternions)

# Add rotations
#add_subdirectory(rotations)

# Add positions
#add_subdirectory(positions)

# Add poses
#add_subdirectory(poses)

# Add test
if(BUILD_GTEST)
	add_subdirectory(test)
endif()


# Add Doxygen documentation
add_subdirectory(doc/doxygen)

ADD_CUSTOM_TARGET(demo DEPENDS demo_rotations)

# Generate FindKindr.cmake file
file(WRITE cmake/FindKindr.cmake
"# This file was automatically generated during the installation of the Kindr library
# and can be used through cmake to find the corresponding header files. A copy of this
# file was created in ${CMAKE_ROOT}/Modules (depending on the CMAKE_ROOT variable).

set(Kindr_INCLUDE_DIRS
${CMAKE_INSTALL_PREFIX}/include/Kindr/include
)
set(Kindr_FOUND TRUE)
#message(\"-- Kindr found (include: ${CMAKE_INSTALL_PREFIX}/include/Kindr/include)\")
"
)

# Setting for make install
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/include/Kindr)")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX}/include/Kindr)
install(FILES cmake/FindKindr.cmake DESTINATION ${CMAKE_ROOT}/Modules)

