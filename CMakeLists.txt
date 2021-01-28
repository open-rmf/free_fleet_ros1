cmake_minimum_required(VERSION 3.5.0)
project(free_fleet_ros1 VERSION 1.0.0 LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
  geometry_msgs
)

find_package(free_fleet REQUIRED)
find_package(free_fleet_cyclonedds REQUIRED)

catkin_package()

file(GLOB lib_srcs
  "src/agv/*.cpp"
  "src/ros1/*.cpp"
  "src/*.cpp"
)

add_library(free_fleet_ros1 SHARED
  ${lib_srcs}
)
target_link_libraries(free_fleet_ros1
  free_fleet::free_fleet
  ${catkin_LIBRARIES}
)
target_include_directories(free_fleet_ros1
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)

install(TARGETS free_fleet_ros1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_executable(client
  "src/client/main.cpp"
)
target_link_libraries(client
  free_fleet_ros1
  free_fleet::free_fleet
  free_fleet_cyclonedds::free_fleet_cyclonedds
  ${catkin_LIBRARIES}
)
target_include_directories(client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)

install(
  TARGETS
    client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(testing_targets
  fake_action_server
)
foreach(target ${testing_targets})
  add_executable(${target}
    src/tests/${target}.cpp
  )
  target_link_libraries(${target}
    free_fleet::free_fleet  
    ${catkin_LIBRARIES}
  )
  target_include_directories(${target}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
      ${catkin_INCLUDE_DIRS}
  )
endforeach()

install(
  TARGETS
    ${testing_targets}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
