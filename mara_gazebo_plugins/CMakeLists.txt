cmake_minimum_required(VERSION 3.5)
project(mara_gazebo_plugins)

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
find_package(ament_index_cpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hrim_actuator_rotaryservo_msgs REQUIRED)
find_package(hrim_actuator_rotaryservo_srvs REQUIRED)
find_package(hrim_actuator_rotaryservo_actions REQUIRED)
find_package(hrim_generic_msgs REQUIRED)
find_package(hrim_generic_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${hrim_actuator_rotaryservo_msgs_INCLUDE_DIRS}
  ${hrim_actuator_rotaryservo_srvs_INCLUDE_DIRS}
  ${hrim_generic_msgs_INCLUDE_DIRS}
  ${hrim_generic_srvs_INCLUDE_DIRS}
)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# mara_gazebo_joint_plugin
add_library(mara_gazebo_joint_plugin SHARED
  src/mara_gazebo_joint_plugin.cpp
  src/spline.cpp
)
ament_target_dependencies(mara_gazebo_joint_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "rclcpp_action"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "hrim_actuator_rotaryservo_msgs"
  "hrim_actuator_rotaryservo_srvs"
  "hrim_actuator_rotaryservo_actions"
  "trajectory_msgs"
  "hrim_generic_msgs"
  "hrim_generic_srvs"
  "ament_index_cpp"
)
ament_export_libraries(mara_gazebo_joint_plugin)

# mara_gazebo_joint_training_plugin
add_library(mara_gazebo_joint_training_plugin SHARED
  src/mara_gazebo_joint_training_plugin.cpp
  src/spline.cpp
)
ament_target_dependencies(mara_gazebo_joint_training_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "rclcpp_action"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "hrim_actuator_rotaryservo_msgs"
  "hrim_actuator_rotaryservo_srvs"
  "hrim_actuator_rotaryservo_actions"
  "trajectory_msgs"
  "hrim_generic_msgs"
  "hrim_generic_srvs"
  "ament_index_cpp"
)
ament_export_libraries(mara_gazebo_joint_training_plugin)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  mara_gazebo_joint_plugin
  mara_gazebo_joint_training_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
