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


rosidl_target_interfaces(
  lf
  ${PROJECT_NAME}_msg
  "rosidl_typesupport_cpp"
)

# # ament_target_dependencies(
# # ${PROJECT_NAME}
# # rclcpp
# # )
# target_include_directories(
# lf
# PUBLIC
# ${EIGEN3_INCLUDE_DIR}
# ${rclcpp_INCLUDE_DIRS}
# ${CMAKE_CURRENT_BINARY_DIR}/msg/rosidl_generator_c__visibility_control
# # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# # $<INSTALL_INTERFACE:include>
# )

# target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
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

# ament_export_dependencies(rosidl_default_runtime)
# ament_export_dependencies(${dependencies})
ament_package()
