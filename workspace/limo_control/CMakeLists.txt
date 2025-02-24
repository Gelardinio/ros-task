cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(limo_control src/limo_controller.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)

ament_target_dependencies(limo_control
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(
    TARGETS
    limo_control
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()