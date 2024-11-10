cmake_minimum_required(VERSION 3.5)
project(line_finder)

set(CMAKE_BUILD_TYPE Debug)

set(PACKAGE_SOURCES
  src/line_extraction_node.cpp
  src/line.cpp
  src/line_extraction.cpp
  src/line_extraction_ros.cpp
)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msg
  "msg/ClosestPointToLineSegment.msg"
  "msg/ClosestPointToLineSegmentList.msg"
  "msg/LineSegment.msg"
  "msg/LineSegmentList.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}_msg
  "rosidl_typesupport_cpp"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${PROJECT_BINARY_DIR}/rosidl_generator_cpp
)

add_executable(lf ${PACKAGE_SOURCES})

target_include_directories(lf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  lf
  Eigen3
  geometry_msgs
  rclcpp
  rosidl_default_generators
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

target_link_libraries(lf
  "${cpp_typesupport_target}"
)
# install(DIRECTORY include/
# DESTINATION include/
# )

install(
TARGETS lf
DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

# ament_export_include_directories(include)

# ament_export_dependencies(rosidl_default_runtime)
# ament_export_dependencies(${dependencies})
ament_package()
