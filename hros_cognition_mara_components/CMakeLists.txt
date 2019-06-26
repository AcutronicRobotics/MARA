cmake_minimum_required(VERSION 3.5)

project(hros_cognition_mara_components)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rttest REQUIRED)
find_package(hrim_generic_msgs REQUIRED)
find_package(hrim_actuator_rotaryservo_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include
  ${rviz_yaml_cpp_vendor_INCLUDE_DIRS}
)

# Declare a C++ executable
set(${PROJECT_NAME}_SOURCES
  src/main.cpp
  src/HROSCognitionMaraComponents.cpp
  src/TopicCommand.cpp
  src/TopicState.cpp
)

add_executable(${PROJECT_NAME}
        ${${PROJECT_NAME}_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rttest"
  "hrim_generic_msgs"
  "rclcpp"
  "trajectory_msgs"
  "control_msgs"
  "hrim_actuator_rotaryservo_msgs"
  "yaml_cpp_vendor"
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  stdc++fs
  ${YAML_CPP_LIBRARIES}
)


install(  TARGETS
            ${PROJECT_NAME}
          DESTINATION
            lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  config/motors.yaml
  config/two_motors.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
