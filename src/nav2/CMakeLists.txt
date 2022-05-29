cmake_minimum_required(VERSION 3.8)
project(nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(robot_localization REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(subscribe_initialpose src/subscribe_initialpose.cpp)
ament_target_dependencies(subscribe_initialpose rclcpp geometry_msgs)

install(TARGETS
  subscribe_initialpose
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY src launch param
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
