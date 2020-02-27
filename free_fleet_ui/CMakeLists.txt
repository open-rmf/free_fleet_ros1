cmake_minimum_required(VERSION 3.5.0)
project(free_fleet_ui VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wshadow -Wextra")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(yaml-cpp REQUIRED)
find_package(free_fleet REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# -----------------------------------------------------------------------------

add_executable(free_fleet_ui
  src/main.cpp
  src/FreeFleetUI.cpp
  src/Viewer.cpp
  src/MapConfig.cpp
  src/FleetSelector.cpp
  src/RobotSelector.cpp
  src/RobotRequester.cpp
)
target_link_libraries(free_fleet_ui
  Qt5::Widgets
  yaml-cpp
  ${free_fleet_LIBRARIES}
)
target_include_directories(free_fleet_ui
  PRIVATE
    ${free_fleet_INCLUDE_DIRS}
)

install(
  TARGETS free_fleet_ui
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
