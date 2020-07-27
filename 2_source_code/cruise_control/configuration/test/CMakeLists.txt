cmake_minimum_required(VERSION 3.5)

project(test_configuration_library VERSION 1.0.0 LANGUAGES CXX)

# Locate GTest
find_package(GTest REQUIRED)

set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

include(GNUInstallDirs)

set ( PROJECT_LINK_LIBS 
${CMAKE_BINARY_DIR}/configuration/libconfiguration_library.a 
${GTEST_LIBRARIES} 
pthread)

include_directories(${CMAKE_SOURCE_DIR}/configuration/header)

add_executable(test_configuration_library cruise_configuration_test.cpp)

target_link_libraries(test_configuration_library ${PROJECT_LINK_LIBS} )

