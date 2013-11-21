
cmake_minimum_required (VERSION 2.8)

#--------------------------#
# Documentation
SET(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)


FIND_PACKAGE(Doxygen)
IF(DOXYGEN)
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  SET(DOXYGEN_IMAGE_DIRS "\"${DOCSOURCE_DIR}/figs/\"")
  SET(DOXYGEN_SOURCE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/doc\" \"${CMAKE_CURRENT_SOURCE_DIR}/../rm_rotations/doc\" \"${CMAKE_CURRENT_SOURCE_DIR}/../rm_rotations/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../rm_common/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../rm_quaternions/doc\" \"${CMAKE_CURRENT_SOURCE_DIR}/../rm_quaternions/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../rm_linear_algebra/include/\" ")

  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  SET(DOXYGEN_QUIET YES)
  MAKE_DIRECTORY(${HTML_DIR})
  SET( HAVE_DOT YES )
  
  CONFIGURE_FILE(${DOCSOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)
  
  ADD_CUSTOM_TARGET(doc ${DOXYGEN} --silent ${DOCSOURCE_DIR}/doxygen.config)
ENDIF(DOXYGEN)

#-------------------------#