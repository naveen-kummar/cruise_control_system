
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

file(GLOB SOURCES "src/basic_cruise_control.cpp", 
"src/concrete_strategy_decorator.cpp", 
"src/jerk_control_decorator.cpp", 
"src/cruise_algo_factory.cpp")

set ( PROJECT_LINK_LIBS ${CMAKE_BINARY_DIR}/configuration/libconfiguration_library.a)

include_directories(${CMAKE_SOURCE_DIR}/configuration/header)
 
#Generate the static library from the sources
add_library(cruise_algo_library STATIC ${SOURCES})

target_link_libraries(cruise_algo_library ${PROJECT_LINK_LIBS})

install(TARGETS cruise_algo_library DESTINATION ${CMAKE_BINARY_DIR})

