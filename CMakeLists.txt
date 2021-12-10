cmake_minimum_required(VERSION 3.5.0)
project(free_fleet_ros1 VERSION 1.0.0 LANGUAGES C CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
  tf2
  roscpp
  tf2_ros
  nav_msgs
  actionlib
  sensor_msgs
  geometry_msgs
  move_base_msgs
  tf2_geometry_msgs
  catch_ros
)

find_package(rmf_utils REQUIRED)
find_package(free_fleet REQUIRED)
find_package(free_fleet_cyclonedds REQUIRED)

catkin_package()

# ------------------------------------------------------------------------------

file(GLOB lib_srcs
  "src/free_fleet_ros1/client/*.cpp"
  # "src/free_fleet_ros1/server/*.cpp"
)

add_library(free_fleet_ros1 SHARED
  ${lib_srcs}
)
target_link_libraries(free_fleet_ros1
  rmf_utils::rmf_utils
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

# ------------------------------------------------------------------------------

# add_executable(client
#   "src/client/main.cpp"
# )
# target_link_libraries(client
#   free_fleet_ros1
#   free_fleet::free_fleet
#   free_fleet_cyclonedds::free_fleet_cyclonedds
#   ${catkin_LIBRARIES}
# )
# target_include_directories(client
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
#   PRIVATE
#     ${catkin_INCLUDE_DIRS}
# )
# 
# install(
#   TARGETS
#     client
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ------------------------------------------------------------------------------
if (BUILD_TESTING)
  file(GLOB_RECURSE test_srcs "test/*.cpp")
  catch_add_test(test_free_fleet_ros1
    ${test_srcs}
  )
  target_link_libraries(test_free_fleet_ros1
    free_fleet_ros1
    ${catkin_LIBRARIES}
  )
  target_include_directories(test_free_fleet_ros1
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${catkin_INCLUDE_DIRS}
  )
  install(TARGETS test_free_fleet_ros1
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
	)
endif()

# ------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )
