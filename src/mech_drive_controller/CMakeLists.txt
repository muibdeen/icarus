cmake_minimum_required(VERSION 3.8)
project(mech_drive_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rcpputils REQUIRED)

add_library(mech_drive_controller SHARED
  src/mech_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_include_directories(mech_drive_controller PRIVATE include)
ament_target_dependencies(mech_drive_controller
  builtin_interfaces
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
  rcpputils
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mech_drive_controller PRIVATE "DIFF_DRIVE_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(mech_drive_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface mech_drive_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS mech_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gmock(test_mech_drive_controller
    test/test_mech_drive_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_mech_drive_controller.yaml)
  target_include_directories(test_mech_drive_controller PRIVATE include)
  target_link_libraries(test_mech_drive_controller
    mech_drive_controller
  )

  ament_target_dependencies(test_mech_drive_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(
    test_load_mech_drive_controller
    test/test_load_mech_drive_controller.cpp
  )
  target_include_directories(test_load_mech_drive_controller PRIVATE include)
  ament_target_dependencies(test_load_mech_drive_controller
    controller_manager
  )
  target_compile_definitions(test_load_mech_drive_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  ament_add_gmock(
    test_accumulator
    test/test_accumulator.cpp
  )

  target_include_directories(test_accumulator PRIVATE include)
  ament_target_dependencies(test_accumulator)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_msgs
  rcpputils
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  mech_drive_controller
)
ament_package()
