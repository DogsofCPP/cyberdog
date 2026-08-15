cmake_minimum_required(VERSION 3.8)
project(cyberdog_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
  DESTINATION include/)

install(FILES
    config/fs_machine_config.toml
    DESTINATION lib/${PROJECT_NAME})

if(LOCAL_BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(protocol REQUIRED)
  find_package(cyberdog_common REQUIRED)
  include_directories(include)
  include_directories(test)
  add_executable(heartbeats_test test/heartbeats_test.cpp)
  ament_target_dependencies(heartbeats_test rclcpp protocol)
  ament_add_gtest(fs_machine_test_controller test/fs_machine_test_controller.cpp)
  ament_add_gtest(fs_machine_test_actuator1 test/fs_machine_test_actuator1.cpp)
  ament_add_gtest(fs_machine_test_actuator2 test/fs_machine_test_actuator2.cpp)
  ament_target_dependencies(fs_machine_test_controller rclcpp protocol cyberdog_common)
  ament_target_dependencies(fs_machine_test_actuator1 rclcpp protocol cyberdog_common)
  ament_target_dependencies(fs_machine_test_actuator2 rclcpp protocol cyberdog_common)
  target_compile_definitions(fs_machine_test_controller PRIVATE BenchmarkPath="${CMAKE_INSTALL_PREFIX}/lib/cyberdog_machine")
  target_compile_definitions(fs_machine_test_actuator1 PRIVATE BenchmarkPath="${CMAKE_INSTALL_PREFIX}/lib/cyberdog_machine")
  target_compile_definitions(fs_machine_test_actuator2 PRIVATE BenchmarkPath="${CMAKE_INSTALL_PREFIX}/lib/cyberdog_machine")
  install(TARGETS
    heartbeats_test fs_machine_test_controller fs_machine_test_actuator1  fs_machine_test_actuator2
    DESTINATION lib/${PROJECT_NAME})
  install(FILES
    test/fs_machine_test_config.toml
    DESTINATION lib/${PROJECT_NAME})
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories()
ament_package()
