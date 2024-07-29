cmake_minimum_required(VERSION 3.5)
project(op3_description)

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
find_package(xacro REQUIRED)


install(
  DIRECTORY launch urdf rviz worlds config meshes
  DESTINATION share/${PROJECT_NAME}
)


ament_package()