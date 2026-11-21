cmake_minimum_required(VERSION 3.8)
project(robot_state_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vida_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(robot_state_ekf_node_exec src/robot_state_ekf_node.cpp)

target_include_directories(robot_state_ekf_node_exec PRIVATE ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(robot_state_ekf_node_exec
  std_srvs
  rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs vida_interfaces
)

install(TARGETS robot_state_ekf_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
