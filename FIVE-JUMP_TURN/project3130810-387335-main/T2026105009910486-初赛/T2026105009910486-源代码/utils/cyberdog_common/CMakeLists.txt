cmake_minimum_required(VERSION 3.8)
project(cyberdog_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # 暂时使用-Wno-class-memaccess屏蔽掉rapidjson的编译警告
  # 该警告来自于高等级(>=9)的gcc检查
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-class-memaccess)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rapidjson REQUIRED)
find_package(toml REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lcm REQUIRED)
find_package(xpack REQUIRED)

set(dependencies
  toml
  rapidjson
  rclcpp
  std_srvs
  lcm
  xpack
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(cyberdog_log SHARED
  include/cyberdog_common/cyberdog_log.hpp
  src/cyberdog_log.cpp)

target_include_directories(cyberdog_log PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(cyberdog_log
  PRIVATE "CYBERDOG_LOG_DLL")
ament_target_dependencies(
  cyberdog_log
  ${dependencies})

add_library(cyberdog_fds SHARED
  src/cyberdog_fds_impl.cpp)
target_include_directories(cyberdog_fds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  cyberdog_fds
  ${dependencies})
link_directories(/usr/local/lib)
target_link_libraries(cyberdog_fds galaxy-fds-sdk-cpp PocoNet PocoNetSSL
  PocoFoundation PocoJSON crypto)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(DIRECTORY include/
  DESTINATION include/)

install(
  TARGETS cyberdog_log
  EXPORT cyberdog_log
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS cyberdog_fds
  EXPORT cyberdog_fds
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if(LOCAL_BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  include_directories(include)
  include_directories(test)
  ament_add_gtest(jsontest test/json_test.cpp src/cyberdog_log.cpp)
  ament_add_gtest(tomltest test/toml_test.cpp src/cyberdog_log.cpp)
  ament_add_gtest(logtest test/log_test.cpp src/cyberdog_log.cpp)
  ament_add_gtest(lcmtest test/lcm_test.cpp src/cyberdog_log.cpp)
  ament_add_gtest(msgqueuetest test/msg_queue_test.cpp src/cyberdog_log.cpp)
  target_link_libraries(lcmtest lcm)
  target_compile_definitions(tomltest PRIVATE BenchmarkPath="${CMAKE_INSTALL_PREFIX}/lib/cyberdog_common")
  target_compile_definitions(jsontest PRIVATE BenchmarkPath="${CMAKE_INSTALL_PREFIX}/lib/cyberdog_common")
  ament_target_dependencies(tomltest rclcpp toml)
  ament_target_dependencies(jsontest rclcpp rapidjson)
  ament_target_dependencies(logtest rclcpp)
  ament_target_dependencies(lcmtest rclcpp xpack)
  ament_target_dependencies(msgqueuetest rclcpp)
  install(TARGETS
    jsontest tomltest logtest lcmtest msgqueuetest
    DESTINATION lib/${PROJECT_NAME})
  install(FILES
    test/benchmark.toml test/benchmark.json test/test_cs.lcm
    DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY
    test/lcmtestcs test/structor
    DESTINATION lib/${PROJECT_NAME})
endif()
set(LOCAL_BUILD_TESTING2 TRUE)
if(LOCAL_BUILD_TESTING2)
  add_executable(test_fds
    test/fds/test_fds.cpp
  )
  target_include_directories(test_fds PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(test_fds PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(test_fds
    rclcpp
  )
  target_link_libraries(test_fds
    cyberdog_fds
    cyberdog_log
  )
endif()

set(LOCAL_BUILD_TESTING3 TRUE)
if(LOCAL_BUILD_TESTING3)
  find_package(protocol REQUIRED)
  add_executable(model_test
    test/model_test/cyberdog_model_test.cpp
  )
  target_include_directories(model_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(model_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(model_test
    rclcpp
    ament_index_cpp
    protocol
    std_msgs
  )
  target_link_libraries(model_test
  cyberdog_fds
  cyberdog_log)

endif()


ament_export_include_directories(include)
ament_export_targets(cyberdog_log HAS_LIBRARY_TARGET)
ament_export_targets(cyberdog_fds HAS_LIBRARY_TARGET)
# ament_export_libraries(cyberdog_log)
ament_export_dependencies(${dependencies})
ament_package()
