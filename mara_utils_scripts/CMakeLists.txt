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
  scripts/mara_control.py
  scripts/spawn_mara.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
