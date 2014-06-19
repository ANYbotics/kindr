# - Find ROS
# This module finds an installed ROS package.
#
# It sets the following variables:
#  ROS_FOUND       - Set to false, or undefined, if ROS isn't found.

IF(UNIX)
	FIND_PROGRAM(ROS_EXEC NAME rosrun PATHS)  
	IF(ROS_EXEC)
        SET(ROS_FOUND TRUE)
	ENDIF(ROS_EXEC)
ENDIF(UNIX)

IF (ROS_FOUND)
   # show which ROS was found only if not quiet
   IF (NOT ROS_FIND_QUIETLY)
      MESSAGE(STATUS "Found ROS")
   ENDIF (NOT ROS_FIND_QUIETLY)
ELSE (ROS_FOUND)
   # warning if ROS is required but not found
   IF (ROS_FIND_REQUIRED)
      MESSAGE(WARNING "Could not find ROS")
   ENDIF (ROS_FIND_REQUIRED)
ENDIF (ROS_FOUND)

IF (ROS_FOUND)

    # Set node dependencies
    set(rospack_dep roscpp)
    set(rospack_cmd rospack)
    
    # Get ROS includes
    set(rospack_arg cflags-only-I ${rospack_dep})
    execute_process(
      COMMAND ${rospack_cmd} ${rospack_arg}
      OUTPUT_VARIABLE rospack_inc_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE " " ";" ROSPACK_INC_DIR ${rospack_inc_dir})
    
    # Get library folders
    set(rospack_arg libs-only-L ${rospack_dep})
    execute_process(
      COMMAND ${rospack_cmd} ${rospack_arg}
      OUTPUT_VARIABLE rospack_lib_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE " " ";" ROSPACK_LIB_DIR ${rospack_lib_dir})
    
    # Get libraries
    set(rospack_arg libs-only-l ${rospack_dep})
    execute_process(
      COMMAND ${rospack_cmd} ${rospack_arg}
      OUTPUT_VARIABLE rospack_libs
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE " " ";" ROSPACK_LIBS ${rospack_libs})
    
    # Include directories
    include_directories(${ROSPACK_INC_DIR})
    
    # Linking directories
    link_directories(${ROSPACK_LIB_DIR})

ENDIF (ROS_FOUND)