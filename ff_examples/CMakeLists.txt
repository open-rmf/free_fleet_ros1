cmake_minimum_required(VERSION 3.5.0)

project(ff_examples)

#===============================================================================

find_package(catkin QUIET)

catkin_package()

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)
