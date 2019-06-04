cmake_minimum_required(VERSION 3.5)

project(cable_pose_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_BUILD_TYPE DEBUG)

find_package(rmw REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost 1.58 REQUIRED)

# find ROS 1 packages
include(cmake/find_ros1_package.cmake)
include(cmake/find_ros1_interface_packages.cmake)

find_ros1_interface_packages(ros1_message_packages)

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(STATUS "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid being treated as a plain CMake package from ament_tools
  ament_package()
  return()
endif()

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  message(STATUS "Failed to find ROS 1 roscpp, skipping...")
  # call ament_package() to avoid being treated as a plain CMake package from ament_tools
  ament_package()
  return()
endif()

find_ros1_package(geometry_msgs REQUIRED)

set(prefixed_ros1_message_packages "")
foreach(ros1_message_package ${ros1_message_packages})
  find_ros1_package(${ros1_message_package} REQUIRED)
  list(APPEND prefixed_ros1_message_packages "ros1_${ros1_message_package}")
endforeach()


ament_package()

if(NOT WIN32)
  # ignore warning in ROS 1 message headers
  set_source_files_properties(${generated_files}
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter")
endif()


add_executable(${PROJECT_NAME}_actions
        src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_actions
    "rclcpp"
    "geometry_msgs"
    ${prefixed_ros1_message_packages}
    ${ros2_message_packages}
    "ros1_roscpp"
    "ros1_geometry_msgs"
)

install(TARGETS ${PROJECT_NAME}_actions
  DESTINATION lib/${PROJECT_NAME})
