cmake_minimum_required(VERSION 3.8)
project(laser_lines)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(behaviortree_cpp_v3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSegment.msg"
  "msg/LineSegmentList.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rclcpp rosidl_default_runtime)

add_executable(
  bt 
  # src/laser_lines.cpp
  src/line_extraction_node.cpp
  src/line.cpp
  src/line_extraction.cpp
  src/line_extraction_ros.cpp
)

ament_target_dependencies(bt ${catkin_LIBRARIES})

rosidl_target_interfaces(
  bt
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
# rosidl_get_typesupport_target(
#   cpp_typesupport_target
#   ${PROJECT_NAME}
#   "rosidl_typesupport_cpp"
# )


ament_target_dependencies(
  bt
  rclcpp
)

target_include_directories(
  bt
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(bt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  bt
  # "behaviortree_cpp_v3"
  Eigen3
  "rclcpp"
  "rosidl_default_generators"
  "sensor_msgs"
  "visualization_msgs"
  # "geometry_msgs"
  # "nav_msgs"
  # "tf2_ros"
)

install(TARGETS bt
DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
