cmake_minimum_required(VERSION 3.8)
project(mid360_live)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(livox_cloud_converter src/livox_cloud_converter.cpp)
target_compile_features(livox_cloud_converter PRIVATE cxx_std_14)
target_compile_options(livox_cloud_converter PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(livox_cloud_converter rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS livox_cloud_converter DESTINATION lib/${PROJECT_NAME})
ament_package()
