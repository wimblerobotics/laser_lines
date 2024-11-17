cmake_minimum_required(VERSION 3.10)
project(line_finder)

set(CMAKE_BUILD_TYPE Debug)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

set(SOURCES_MOTOR_CHARACTERIZATION
  src/line_extraction_node.cpp
  src/line.cpp
  src/line_extraction.cpp
  src/line_extraction_ros.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClosestPointToLineSegment.msg"
  "msg/ClosestPointToLineSegmentList.msg"
  "msg/LineSegment.msg"
  "msg/LineSegmentList.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${PROJECT_BINARY_DIR}/rosidl_generator_cpp
)

add_executable(lf ${SOURCES_MOTOR_CHARACTERIZATION})

target_include_directories(lf PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(
  lf
  Eigen3
  rclcpp
  rosidl_default_generators
  sensor_msgs
  std_msgs
  visualization_msgs
)

target_link_libraries(lf
  "${cpp_typesupport_target}"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(lf "${cpp_typesupport_target}")

install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS lf
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)

ament_package()
