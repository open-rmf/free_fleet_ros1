cmake_minimum_required(VERSION 3.5.0)

project(ff_client LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

#===============================================================================

find_package(catkin QUIET COMPONENTS
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
)

find_package(free_fleet REQUIRED)

catkin_package()

#=============================================================================

add_executable(ff_client_node
  src/main.cpp
  src/utilities.cpp
  src/ClientNode.cpp
  src/ClientNodeConfig.cpp
)
target_link_libraries(ff_client_node
  ${free_fleet_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(ff_client_node
  PRIVATE
    ${free_fleet_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

#=============================================================================

set(testing_targets
  fake_action_server
)

foreach(target ${testing_targets})
  add_executable(${target}
    src/tests/${target}.cpp
  )
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
  )
  target_include_directories(${target}
    PRIVATE
      ${catkin_INCLUDE_DIRS}
  )
endforeach()

#=============================================================================

install(
  TARGETS
    ff_client_node
    ${testing_targets}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
