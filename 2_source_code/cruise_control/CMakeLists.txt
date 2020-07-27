cmake_minimum_required(VERSION 3.5)
project(cruise_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Excute this cmake to run unit test of 'cruise configuration library' and 'algorithm library'
add_subdirectory(configuration)
add_subdirectory(configuration/test)
add_subdirectory(algorithm)
add_subdirectory(algorithm/test)






