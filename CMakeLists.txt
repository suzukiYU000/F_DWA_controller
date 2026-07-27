cmake_minimum_required(VERSION 3.8)
project(f_dwa_controller)

find_package(ament_cmake REQUIRED)
find_package(dwb_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

set(library_name f_dwa_controller)

add_library(${library_name} SHARED
  src/certified_dwb_local_planner.cpp
)

target_compile_features(${library_name} PUBLIC cxx_std_17)
target_include_directories(${library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  dwb_core
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core f_dwa_controller.xml)

install(
  TARGETS ${library_name}
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright treats every Git repository root as requiring a
  # license-only CONTRIBUTING.md. The repository already has an MIT LICENSE
  # and all source files carry matching headers.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_targets(export_${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  dwb_core
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

ament_package()
