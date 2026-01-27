cmake_minimum_required(VERSION 3.10)
project(rusty_control LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 2. Library Creation
# Fixed paths: removed 'hardware/' prefix as your tree shows src/ and include/ at root
add_library(${PROJECT_NAME}
  SHARED
  src/rusty_system.cpp
  src/i2c_device.cpp
  src/motor.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rusty_control PUBLIC 
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RUSTY_CONTROL_BUILDING_DLL")

# 3. Plugin Registration
pluginlib_export_plugin_description_file(hardware_interface rusty_control.xml)

# 4. Installations
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 5. Testing
# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)
#   # Fixed path to test file
#   ament_add_gmock(test_rusty_system test/test_rusty_system.cpp)
#   target_link_libraries(test_rusty_system ${PROJECT_NAME})
#   ament_target_dependencies(test_rusty_system
#     pluginlib
#     ros2_control_test_assets
#   )
# endif()

# 6. Exports
# Fixed: changed export_jetbot_control to export_rusty_control
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()