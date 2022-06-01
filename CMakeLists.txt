cmake_minimum_required(VERSION 3.8)
project(f4f_tools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

## --------------------------------------------------------------
## |                            build                           |
## --------------------------------------------------------------

add_executable(talker src/fake_lidar_pub.cpp)
ament_target_dependencies(talker rclcpp sensor_msgs)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS talker
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
