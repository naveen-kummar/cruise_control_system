cmake_minimum_required(VERSION 3.5)
project(cruise_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

include_directories(header)

file(GLOB SOURCES 
  "src/cruise_configuration.cpp")
 
#Generate the static library from the sources
add_library(configuration_library STATIC ${SOURCES})

install(TARGETS configuration_library DESTINATION ${CMAKE_BINARY_DIR})

