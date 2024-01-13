cmake_minimum_required(VERSION 3.10)
project(nmpc_cart_pole)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS -fPIC)

# set(CMAKE_INSTALL_PREFIX /tmp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)
pkg_check_modules(CPPAD REQUIRED cppad)
include_directories(${IPOPT_INCLUDE_DIRS} ${CPPAD_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(proto)
add_subdirectory(control)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.git*" EXCLUDE)

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.json"
    PATTERN "*.git*" EXCLUDE)
