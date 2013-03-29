## Author: Kun Wang <ifreedom.cn@gmail.com>
## Version: $Id: CMakeLists.txt,v 0.0 2013/03/29 02:32:59 ifreedom Exp $

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(SET CMP0003 NEW)

# Search in the `cmake' directory for additional CMake modules.
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake)

#-----------------------------------------------------------------------------#
# Build options

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
    "Choose the type of build, options are:
    Debug Release."
    FORCE)
endif()

# Search for C header files in these directories.
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_BINARY_DIR}/include)
# Put libraries into `lib'.

if(NOT WIN32)
  set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
endif()

function(msvc_force_static_link_runtime)
  foreach(flag_var

	  CMAKE_CXX_FLAGS
	  CMAKE_CXX_FLAGS_DEBUG
	  CMAKE_CXX_FLAGS_RELEASE
	  CMAKE_CXX_FLAGS_MINSIZEREL
	  CMAKE_CXX_FLAGS_RELWITHDEBINFO
	  CMAKE_C_FLAGS_DEBUG
	  CMAKE_C_FLAGS_RELEASE
	  CMAKE_C_FLAGS_MINSIZEREL
	  CMAKE_C_FLAGS_RELWITHDEBINFO
	  )

	if(${flag_var} MATCHES "/MD")
	  string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
	endif()

	if(${flag_var} MATCHES "/MDd")
	  string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
	endif()
  endforeach(flag_var)
endfunction()

# Set the project name.
project(serialio)

if (MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
	add_definitions(-DNO_MIN_MAX)

	#We statically link to reduce dependancies
	msvc_force_static_link_runtime()
endif()

set(_VERSION 0.1)

set(_srcs src/serialio.c)
set(_headers include/serialio.h)

add_definitions(-DSIO_EXPORT)
add_library(serialio STATIC ${_srcs} ${_headers})

add_executable(test test/test.c)
target_link_libraries(test serialio)

### CMakeLists.txt ends here
