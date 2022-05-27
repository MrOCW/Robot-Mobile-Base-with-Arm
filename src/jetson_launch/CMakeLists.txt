cmake_minimum_required(VERSION 3.5)
project(jetson_launch)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(subscribe_initialpose src/subscribe_initialpose.cpp)
ament_target_dependencies(subscribe_initialpose rclcpp geometry_msgs)

install(TARGETS
  subscribe_initialpose
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  src launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
