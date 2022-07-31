cmake_minimum_required(VERSION 3.21.0)
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