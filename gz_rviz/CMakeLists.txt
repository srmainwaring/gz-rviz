cmake_minimum_required(VERSION 3.5)
project(gz_rviz)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

# Qt packages
find_package(Qt5 COMPONENTS
  Core
  Quick
  QuickControls2
  REQUIRED
)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Gazebo libraries
find_package(gz_rviz_common REQUIRED)
find_package(gz_rviz_plugins REQUIRED)
find_package(gz-math7 REQUIRED)

# Default to Garden
find_package(gz-gui7 REQUIRED)
set(GZ_GUI_VER ${gz-gui7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

message(STATUS "Compiling against Gazebo Garden")

qt5_add_resources(resources_RCC include/gz/rviz/rviz.qrc)

include_directories(SYSTEM
  ${GZ-COMMON_INCLUDE_DIRS}
  ${GZ-GUI_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
)

link_directories(
  ${GZ-COMMON_LIBRARY_DIRS}
  ${GZ-GUI_LIBRARY_DIRS}
  ${GZ-MATH_LIBRARY_DIRS}
)

include_directories(include)

add_executable(gz_rviz
  src/rviz/application.cpp
  src/rviz/rviz.cpp
  include/gz/rviz/rviz.hpp
  ${resources_RCC}
)

ament_target_dependencies(gz_rviz
  ament_index_cpp
  gz_rviz_common
  gz_rviz_plugins
  gz-math7
  gz-gui${GZ_GUI_VER}
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_msgs
  tf2_ros
  visualization_msgs
)

target_link_libraries(gz_rviz Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

install(TARGETS gz_rviz
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
