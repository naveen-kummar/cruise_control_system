cmake_minimum_required(VERSION 3.5)
project(cruise_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include(GNUInstallDirs)

#Temporary static library of cruise configuration and algorithm stored here
LINK_DIRECTORIES(${CMAKE_BINARY_DIR})

########### Build Cruise Configuration as static library - start #########
include_directories(cruise_control/configuration/header)

file(GLOB SOURCES 
  "cruise_control/configuration/src/cruise_configuration.cpp")


#Generate the static library from the sources
add_library(configuration_library STATIC ${SOURCES})

########### Build Cruise Configuration as static library  - End ####

########### Build Algorithmas static library - Start #############

include_directories(cruise_control/algorithm/header)

file(GLOB SOURCES 
  "cruise_control/algorithm/src/basic_cruise_control.cpp"
  "cruise_control/algorithm/src/concrete_strategy_decorator.cpp"
  "cruise_control/algorithm/src/jerk_control_decorator.cpp"
  "cruise_control/algorithm/src/cruise_algo_factory.cpp")


#Generate the static library from the sources
add_library(algo_library STATIC ${SOURCES})

########### Build Algorithmas static library - End ##################


########### Build Vehicle stub node - satrt ############
set(SOURCES_VEHICLE
    cruise_control/vehicle_stub/src/main_node.cpp
    cruise_control/vehicle_stub/header/vehicle_stub_runner.h
    cruise_control/vehicle_stub/src/vehicle_stub_runner.cpp
    cruise_control/vehicle_stub/header/vehicle_stub.h
    cruise_control/vehicle_stub/src/vehicle_stub.cpp
)

add_executable(vehicle_stub ${SOURCES_VEHICLE})
ament_target_dependencies(vehicle_stub rclcpp std_msgs)

########### Build Vehicle stub node - End ############

########### Build cruise control node - satrt ############

set ( PROJECT_LINK_LIBS 
${CMAKE_BINARY_DIR}/libalgo_library.a
${CMAKE_BINARY_DIR}/libconfiguration_library.a)

include_directories(${CMAKE_SOURCE_DIR}/cruise_control/algorithm/header)
include_directories(${CMAKE_SOURCE_DIR}/cruise_control/configuration/header)

set(SOURCES_CRUISE
    cruise_control/cruise_control/src/main_node.cpp
    cruise_control/cruise_control/header/cruise_node_runner.h
    cruise_control/cruise_control/src/cruise_node_runner.cpp
    cruise_control/cruise_control/header/cruise_manager.h
    cruise_control/cruise_control/src/cruise_manager.cpp
)
add_executable(cruise_manager ${SOURCES_CRUISE})

ament_target_dependencies(cruise_manager rclcpp std_msgs)
target_link_libraries(cruise_manager ${PROJECT_LINK_LIBS} )

########### Build cruise control node - end ############


install(TARGETS
  vehicle_stub
  cruise_manager
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
