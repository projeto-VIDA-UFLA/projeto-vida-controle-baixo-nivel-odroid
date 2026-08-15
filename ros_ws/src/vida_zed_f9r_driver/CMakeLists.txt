# Copyright 2026 Projeto VIDA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(vida_zed_f9r_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vida_interfaces REQUIRED)

add_library(${PROJECT_NAME}_core
  src/serial_transport.cpp
  src/ubx_parser.cpp
  src/ubx_protocol.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(zed_f9r_node
  src/main.cpp
  src/zed_f9r_node.cpp
)
target_compile_features(zed_f9r_node PUBLIC cxx_std_17)
target_include_directories(zed_f9r_node PRIVATE include)
target_link_libraries(zed_f9r_node ${PROJECT_NAME}_core)
ament_target_dependencies(zed_f9r_node
  rclcpp
  sensor_msgs
  std_msgs
  vida_interfaces
)

install(TARGETS
  ${PROJECT_NAME}_core
  zed_f9r_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_ubx_parser test/test_ubx_parser.cpp)
  target_link_libraries(test_ubx_parser ${PROJECT_NAME}_core)

  ament_add_gtest(test_nav_hpposllh test/test_nav_hpposllh.cpp)
  target_link_libraries(test_nav_hpposllh ${PROJECT_NAME}_core)

  ament_add_gtest(test_esf_raw test/test_esf_raw.cpp)
  target_link_libraries(test_esf_raw ${PROJECT_NAME}_core)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
