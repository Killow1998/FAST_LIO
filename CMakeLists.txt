cmake_minimum_required(VERSION 3.5)
project(fast_lio)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

add_compile_options(-w)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES geometry_msgs
)

# Executable
add_executable(fastlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
)

target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(fastlio_mapping
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  livox_ros_driver2
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fastlio_mapping "${cpp_typesupport_target}")

# Install rules
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
