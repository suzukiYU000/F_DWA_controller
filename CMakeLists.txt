cmake_minimum_required(VERSION 3.8)
project(f_dwa_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

set(library_name f_dwa_controller_core)
set(transport_library_name f_dwa_transport)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandDispatch.msg"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

# rosidl owns the generated f_dwa_controller Python package. Install the design
# helper as a submodule without creating a second same-named Python target.
install(
  FILES python/${PROJECT_NAME}/fir_filter_design.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

add_library(${library_name} SHARED
  src/certified_dwb_local_planner.cpp
  src/dwa_controllers.cpp
  src/fir_input_dynamics.cpp
  src/native_input_dynamics.cpp
  src/native_input_trajectory_generator.cpp
  src/terminal_stop_dynamics.cpp
  src/trajectory_certifier.cpp
)

add_library(${transport_library_name} SHARED
  src/command_delay_node.cpp
  src/command_delay_queue.cpp
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
target_link_libraries(${library_name} "${cpp_typesupport_target}")
target_link_libraries(${transport_library_name} "${cpp_typesupport_target}")

target_compile_features(${library_name} PUBLIC cxx_std_17)
target_compile_features(${transport_library_name} PUBLIC cxx_std_17)
target_include_directories(${library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${transport_library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  dwb_core
  dwb_msgs
  dwb_plugins
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
  tf2_ros
)

ament_target_dependencies(${transport_library_name}
  diagnostic_msgs
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs
)

add_executable(command_delay_transport
  src/command_delay_main.cpp
)
target_compile_features(command_delay_transport PUBLIC cxx_std_17)
target_link_libraries(command_delay_transport ${transport_library_name})
ament_target_dependencies(command_delay_transport rclcpp)

pluginlib_export_plugin_description_file(nav2_core f_dwa_controller.xml)
pluginlib_export_plugin_description_file(
  dwb_core f_dwa_trajectory_generators.xml
)

install(
  TARGETS
    ${library_name}
    ${transport_library_name}
    command_delay_transport
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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright treats every Git repository root as requiring a
  # license-only CONTRIBUTING.md. The repository already has an MIT LICENSE
  # and all source files carry matching headers.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_command_delay_queue
    test/test_command_delay_queue.cpp
  )
  if(TARGET test_command_delay_queue)
    target_link_libraries(test_command_delay_queue ${transport_library_name})
    ament_target_dependencies(test_command_delay_queue geometry_msgs rclcpp)
  endif()

  ament_add_gtest(test_command_delay_node
    test/test_command_delay_node.cpp
  )
  if(TARGET test_command_delay_node)
    target_link_libraries(test_command_delay_node ${transport_library_name})
    ament_target_dependencies(test_command_delay_node
      rclcpp
      std_srvs
    )
  endif()

  ament_add_gtest(test_native_input_dynamics
    test/test_native_input_dynamics.cpp
    src/native_input_dynamics.cpp
  )
  if(TARGET test_native_input_dynamics)
    target_include_directories(test_native_input_dynamics
      PRIVATE
        include
    )
  endif()

  ament_add_gtest(test_fir_input_dynamics
    test/test_fir_input_dynamics.cpp
    src/fir_input_dynamics.cpp
  )
  if(TARGET test_fir_input_dynamics)
    target_include_directories(test_fir_input_dynamics
      PRIVATE
        include
    )
  endif()

  ament_add_gtest(test_native_input_trajectory_generator
    test/test_native_input_trajectory_generator.cpp
  )
  if(TARGET test_native_input_trajectory_generator)
    target_link_libraries(
      test_native_input_trajectory_generator ${library_name}
    )
    ament_target_dependencies(test_native_input_trajectory_generator
      dwb_msgs
      nav_2d_utils
      nav2_util
      rclcpp
    )
    # nav_2d_utils exports plain libraries and upstream dwb_core leaves these
    # symbols unresolved. Resolve their installed paths explicitly and keep
    # them after dwb_core in the executable link order.
    get_filename_component(
      nav_2d_utils_prefix "${nav_2d_utils_DIR}/../../.." ABSOLUTE
    )
    find_library(nav_2d_utils_conversions
      NAMES conversions
      PATHS "${nav_2d_utils_prefix}/lib"
      NO_DEFAULT_PATH
      REQUIRED
    )
    find_library(nav_2d_utils_path_ops
      NAMES path_ops
      PATHS "${nav_2d_utils_prefix}/lib"
      NO_DEFAULT_PATH
      REQUIRED
    )
    find_library(nav_2d_utils_tf_help
      NAMES tf_help
      PATHS "${nav_2d_utils_prefix}/lib"
      NO_DEFAULT_PATH
      REQUIRED
    )
    target_link_libraries(test_native_input_trajectory_generator
      "${nav_2d_utils_conversions}"
      "${nav_2d_utils_path_ops}"
      "${nav_2d_utils_tf_help}"
    )
  endif()

  ament_add_gtest(test_terminal_stop_dynamics
    test/test_terminal_stop_dynamics.cpp
    src/fir_input_dynamics.cpp
    src/native_input_dynamics.cpp
    src/terminal_stop_dynamics.cpp
  )
  if(TARGET test_terminal_stop_dynamics)
    target_include_directories(test_terminal_stop_dynamics
      PRIVATE
        include
    )
  endif()

  ament_add_gtest(test_trajectory_certifier
    test/test_trajectory_certifier.cpp
    src/trajectory_certifier.cpp
  )
  if(TARGET test_trajectory_certifier)
    target_include_directories(test_trajectory_certifier
      PRIVATE
        include
    )
    ament_target_dependencies(test_trajectory_certifier
      geometry_msgs
      nav2_costmap_2d
    )
  endif()

  ament_add_pytest_test(
    test_fir_filter_design
    test/test_fir_filter_design.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} ${transport_library_name})
ament_export_targets(export_${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  diagnostic_msgs
  dwb_core
  dwb_msgs
  dwb_plugins
  nav_2d_msgs
  nav_2d_utils
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rosidl_default_runtime
  std_msgs
  std_srvs
  tf2_ros
)

ament_package()
