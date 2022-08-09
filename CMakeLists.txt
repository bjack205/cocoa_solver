cmake_minimum_required(VERSION 3.16.0)
list(APPEND CMAKE_MESSAGE_CONTEXT cocoa)
project(cocoa VERSION 0.1.0 LANGUAGES C CXX)

# Language Standard
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)
include(CMakePrintHelpers)
include(FetchContent)

#############################################
# Options
#############################################

# Handle default build type
set(REXQUAD_DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified. Setting CMAKE_BUILD_TYPE to ${MOCAP_DEFAULT_BUILD_TYPE}")
  set(CMAKE_BUILD_TYPE ${ULQR_DEFAULT_BUILD_TYPE} CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(COCOA_BUILD_TESTS "Build testing suite for COCOA" ON)
option(COCOA_CODE_COVERAGE "Compile cocoa with Code Coverage." OFF)
option(COCOA_RUN_CLANG_TIDY "Run clang-tidy on the cocoa source files." OFF)

#############################################
# Dependencies
#############################################
add_subdirectory(deps)
add_subdirectory(deps/slap)
AddFmtLib()
AddGoogleTest()
AddQdldl()

#############################################
# Build
#############################################
if(COCOA_CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  message(STATUS "Compiling rsLQR with coverage info.")
  add_compile_options(-O0 -g -fprofile-arcs -ftest-coverage)
  add_link_options(-fprofile-arcs -ftest-coverage)
endif()

# Add compile options
add_compile_options(-Wall -Wextra -pedantic -Werror -Wno-error=unknown-pragmas)
add_compile_options(-mavx2 -mfma)

# Compile src
add_subdirectory(src)

#############################################
# Testing
#############################################
if (${COCOA_BUILD_TESTS})
  add_subdirectory(test)
endif()


##############################
# Documentation
##############################
find_package(Doxygen)
if (DOXYGEN_FOUND)
  message(STATUS "Adding cocoa_docs target for building the documentation.")
  set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/docs/Doxyfile.in)
  set(DOXYGEN_OUT ${PROJECT_SOURCE_DIR}/docs/Doxyfile)
  file(REMOVE ${DOXYGEN_OUT})
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

  add_custom_target(cocoa_docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Build documentation with Doxygen."
    )
else()
  message(STATUS "Couldn't find Doxygen. Try installing using sudo apt-get install doxygen.")
endif()