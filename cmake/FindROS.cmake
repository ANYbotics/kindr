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