cmake_minimum_required(VERSION 3.1.3)
project(moveit)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    rviz_visual_tools
    moveit_visual_tools
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(moveit_node src/moveit.cpp)
target_link_libraries(moveit_node ${catkin_LIBRARIES})
install(TARGETS moveit_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
