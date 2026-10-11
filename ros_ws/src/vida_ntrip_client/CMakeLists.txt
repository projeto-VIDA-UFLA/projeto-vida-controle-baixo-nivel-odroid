# Copyright 2026 Projeto VIDA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(vida_ntrip_client)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}_core
  src/ntrip_connection.cpp
  src/ntrip_protocol.cpp
  src/rtcm_parser.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Threads::Threads)

add_executable(ntrip_client_node
  src/main.cpp
  src/ntrip_client_node.cpp
)
target_compile_features(ntrip_client_node PUBLIC cxx_std_17)
target_include_directories(ntrip_client_node PRIVATE include)
target_link_libraries(ntrip_client_node ${PROJECT_NAME}_core)
ament_target_dependencies(ntrip_client_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  ${PROJECT_NAME}_core
  ntrip_client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_ntrip_protocol test/test_ntrip_protocol.cpp)
  target_link_libraries(test_ntrip_protocol ${PROJECT_NAME}_core)

  ament_add_gtest(test_rtcm_parser test/test_rtcm_parser.cpp)
  target_link_libraries(test_rtcm_parser ${PROJECT_NAME}_core)

  ament_add_gtest(test_ntrip_connection test/test_ntrip_connection.cpp)
  target_link_libraries(test_ntrip_connection ${PROJECT_NAME}_core)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Threads)
ament_package()
