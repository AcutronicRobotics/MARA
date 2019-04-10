cmake_minimum_required(VERSION 3.5)
project(mara_utils_scripts)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
  scripts/mara_joint_control.py
  scripts/spawn_entity.py
  scripts/spawn_mara_arg.py
  scripts/spawn_mara_gripper_85.py
  scripts/spawn_mara_gripper_140.py
  scripts/spawn_mara_gripper_140_camera.py
  scripts/spawn_mara_gripper_hande.py
  scripts/spawn_mara_gripper_ur_j.py
  scripts/spawn_module.py
  scripts/gripper_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
