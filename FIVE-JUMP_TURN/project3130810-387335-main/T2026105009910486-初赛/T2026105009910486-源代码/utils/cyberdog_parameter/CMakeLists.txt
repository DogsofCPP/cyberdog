cmake_minimum_required(VERSION 3.8)
project(cyberdog_parameter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake")
project_initialize_config_parameter()

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(cyberdog_common REQUIRED)
find_package(toml REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs REQUIRED)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(cyberdog_parameter SHARED ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})
target_link_libraries(cyberdog_parameter PUBLIC ${PythonLibs} ${PYTHON_LIBRARIES})
target_include_directories(cyberdog_parameter PUBLIC "${PYTHON_INCLUDE_DIRS}")

target_include_directories(cyberdog_parameter PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>) 

# target_compile_definitions(cyberdog_parameter PRIVATE "CYBERDOG_PARAMETER_DLL")  

ament_target_dependencies(
  cyberdog_parameter
  PUBLIC
  PythonInterp
  PythonLibs
  cyberdog_common
  toml
  rclcpp
  ament_index_cpp
)

install(DIRECTORY include/
  DESTINATION include/)

install(
  TARGETS cyberdog_parameter
  EXPORT cyberdog_parameter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_targets(cyberdog_parameter HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package(CONFIG_EXTRAS
    cmake/FindCython.cmake
    cmake/functions.cmake
)