cmake_minimum_required(VERSION 3.15)
project(vida_odroid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Busca de bibliotecas
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vida_interfaces REQUIRED)


# ----------------- Security -----------------
add_executable(security
    src/security.cpp
    include/security.h
)
target_include_directories(security PRIVATE include)
ament_target_dependencies(security rclcpp std_msgs vida_interfaces)

# ----------------- Install -----------------
install(TARGETS  
  security
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
