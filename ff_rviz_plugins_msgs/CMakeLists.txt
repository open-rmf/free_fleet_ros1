cmake_minimum_required(VERSION 3.5.0)

project(ff_rviz_plugins_msgs)

find_package(catkin QUIET COMPONENTS
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
    Location.msg
    RobotMode.msg
    RobotState.msg
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
