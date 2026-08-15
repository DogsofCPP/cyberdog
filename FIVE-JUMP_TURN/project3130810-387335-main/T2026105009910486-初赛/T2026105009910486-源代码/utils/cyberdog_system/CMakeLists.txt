cmake_minimum_required(VERSION 3.8)
project(cyberdog_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cyberdog_common REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if(LOCAL_BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
  include_directories(include)
  include_directories(test)
  ament_add_gtest(codetest test/robot_code_test.cpp)
  ament_target_dependencies(codetest rclcpp cyberdog_common)
  install(TARGETS
    codetest
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_package()
