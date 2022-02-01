cmake_minimum_required(VERSION 3.5)
project(gz_rviz_plugins)

set(CMAKE_AUTOMOC ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 REQUIRED)

#============================================================================
# Find Tinyxml2
gz_find_package(TINYXML2 REQUIRED PRIVATE PRETTY tinyxml2)

#============================================================================
# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

#============================================================================
# Gazebo libraries
find_package(gz-math7 REQUIRED)

find_package(gz-gui7 REQUIRED)
set(GZ_GUI_VER ${gz-gui7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

message(STATUS "Compiling against Gazebo Garden")

find_package(Qt5Widgets CONFIG REQUIRED)

find_package(gz_rviz_common REQUIRED)

include_directories(include)

include_directories(SYSTEM
  ${GZ-COMMON_INCLUDE_DIRS}
  ${GZ-GUI_INCLUDE_DIRS}
  ${GZ-MATH_INCLUDE_DIRS}
  ${GZ-RENDERING_INCLUDE_DIRS}
)

qt5_add_resources(resources_RCC res/Plugins.qrc)

########################################################################
macro(add_gz_rviz_plugin)
  set(oneValueArgs NAME)
  set(multiValueArgs EXTRA_FILES DEPENDENCIES)
  cmake_parse_arguments(GZ_RVIZ_PLUGIN "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${GZ_RVIZ_PLUGIN_NAME} SHARED ${headers_MOC}
    include/gz/rviz/plugins/${GZ_RVIZ_PLUGIN_NAME}.hpp
    include/gz/rviz/plugins/message_display_base.hpp
    src/rviz/plugins/${GZ_RVIZ_PLUGIN_NAME}.cpp
    ${GZ_RVIZ_PLUGIN_EXTRA_FILES}
    ${resources_RCC}
  )

  ament_target_dependencies(
    ${GZ_RVIZ_PLUGIN_NAME}
    ${GZ_RVIZ_PLUGIN_DEPENDENCIES}
  )

  target_link_libraries(
    ${GZ_RVIZ_PLUGIN_NAME}
    Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2
  )

  target_include_directories(${GZ_RVIZ_PLUGIN_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Qt5Widgets_INCLUDE_DIRS}
    ${GZ-GUI_INCLUDE_DIRS}
  )
endmacro()

########################################################################
add_gz_rviz_plugin(
  NAME AxesDisplay
  DEPENDENCIES
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME GlobalOptions
  DEPENDENCIES
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME GPSDisplay
  DEPENDENCIES
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    sensor_msgs
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME ImageDisplay
  DEPENDENCIES
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    sensor_msgs
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME LaserScanDisplay
  DEPENDENCIES
    tf2_ros
    sensor_msgs
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    gz_rviz_common
)

########################################################################
add_gz_rviz_plugin(
  NAME MarkerDisplay
  EXTRA_FILES
    src/rviz/plugins/MarkerManager.cpp
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    tf2_ros
    visualization_msgs
)

########################################################################
add_gz_rviz_plugin(
  NAME MarkerArrayDisplay
  EXTRA_FILES
    src/rviz/plugins/MarkerManager.cpp
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    tf2_ros
    visualization_msgs
)

########################################################################
add_gz_rviz_plugin(
  NAME PathDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    nav_msgs
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME PointStampedDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME PolygonDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME PoseDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME PoseArrayDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME RobotModelDisplay
  DEPENDENCIES
    gz_rviz_common
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    std_msgs
    tf2_ros
    urdf
)

########################################################################
add_gz_rviz_plugin(
  NAME TFDisplay
  DEPENDENCIES
    tf2_ros
    gz-gui${GZ_GUI_VER}
    gz-math7
    gz-rendering${GZ_RENDERING_VER}
    gz_rviz_common
)

########################################################################
add_library(gz_rviz_plugins SHARED
  "include/gz/rviz/plugins/message_display_base.hpp"
)

ament_target_dependencies(gz_rviz_plugins
  gz_rviz_common
  gz-gui${GZ_GUI_VER}
  rclcpp
  tf2_ros
)

target_include_directories(gz_rviz_plugins
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${GZ-GUI_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_export_libraries(gz_rviz_plugins)

install(
  TARGETS gz_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

########################################################################
ament_export_libraries(
  AxesDisplay
  GlobalOptions
  GPSDisplay
  ImageDisplay
  LaserScanDisplay
  MarkerDisplay
  MarkerArrayDisplay
  PathDisplay
  PointStampedDisplay
  PolygonDisplay
  PoseDisplay
  PoseArrayDisplay
  RobotModelDisplay
  TFDisplay
)

install(
  TARGETS
    AxesDisplay
    GlobalOptions
    GPSDisplay
    ImageDisplay
    LaserScanDisplay
    MarkerDisplay
    MarkerArrayDisplay
    PathDisplay
    PointStampedDisplay
    PolygonDisplay
    PoseDisplay
    PoseArrayDisplay
    RobotModelDisplay
    TFDisplay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
