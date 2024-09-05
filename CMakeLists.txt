cmake_minimum_required(VERSION 3.8)
project(robot_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(odom src/odom.cpp)
ament_target_dependencies(odom rclcpp std_msgs nav_msgs tf2 geometry_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  odom
  DESTINATION lib/${PROJECT_NAME})

ament_package()
