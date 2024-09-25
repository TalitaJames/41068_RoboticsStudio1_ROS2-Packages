cmake_minimum_required(VERSION 3.8)
project(turtlebot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp  REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(OpenCV  REQUIRED)

add_executable(scan_to_image src/scan_to_image.cpp)
ament_target_dependencies(scan_to_image rclcpp sensor_msgs geometry_msgs OpenCV)

install(TARGETS scan_to_image
 DESTINATION lib/${PROJECT_NAME})

# include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

ament_package()
