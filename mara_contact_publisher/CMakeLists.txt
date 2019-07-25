cmake_minimum_required(VERSION 3.5)
project(mara_contact_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# Declare a C++ executable
set(${PROJECT_NAME}_SOURCES
  src/ContactPublisher.cpp
)

add_executable(${PROJECT_NAME}
  ${${PROJECT_NAME}_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "gazebo_dev"
  "gazebo_ros"
  "gazebo_msgs"
  "geometry_msgs"
)

install(TARGETS
  ${PROJECT_NAME}
DESTINATION
  lib/${PROJECT_NAME}
)

ament_package()
