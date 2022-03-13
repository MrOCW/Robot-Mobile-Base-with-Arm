cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

install(
DIRECTORY launch maps param rviz
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
