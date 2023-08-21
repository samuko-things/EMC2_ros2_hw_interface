cmake_minimum_required(VERSION 3.16)
project(emc2_ros2_arduino_driver_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  emc2_ros2_arduino_driver_interface
  SHARED
  hardware/emc2_system.cpp
)
target_compile_features(emc2_ros2_arduino_driver_interface PUBLIC cxx_std_17)
target_include_directories(emc2_ros2_arduino_driver_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/emc2_ros2_arduino_driver_interface>
)
ament_target_dependencies(
  emc2_ros2_arduino_driver_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(emc2_ros2_arduino_driver_interface PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "EMC2_ROS2_ARDUINO_DRIVER_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface emc2_ros2_arduino_driver_interface.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/emc2_ros2_arduino_driver_interface
)

install(TARGETS emc2_ros2_arduino_driver_interface
  EXPORT export_emc2_ros2_arduino_driver_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_emc2_ros2_arduino_driver_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
