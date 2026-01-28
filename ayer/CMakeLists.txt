cmake_minimum_required(VERSION 3.8)
project(grid_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directory
include_directories(
  include
)

add_executable(mapping
  src/mapping_node.cpp
  src/mapping.cpp
  )
target_include_directories(mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mapping PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(mapping
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
)

install(TARGETS
  mapping
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
