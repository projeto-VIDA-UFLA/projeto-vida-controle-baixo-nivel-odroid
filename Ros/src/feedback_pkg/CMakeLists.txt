cmake_minimum_required(VERSION 3.15)
project(feedback_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(WIRINGPI_LIB /usr/lib/aarch64-linux-gnu/libwiringPi.so)
set(WIRINGPI_DEV_LIB /usr/lib/aarch64-linux-gnu/libwiringPiDev.so)

# Busca de bibliotecas
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vida_interfaces)


# ----------------- SendDataControl -----------------
add_executable(get_sensor_data
    src/get_sensor_data/get_sensor_data.cpp
    src/get_sensor_data/get_sensor_data.h
    ../../../Utils/Serial/Serial.cpp
    ../../../Utils/Serial/Serial.h
    ../../../Utils/Files/DefinePin.h
)
ament_target_dependencies(get_sensor_data rclcpp std_msgs)
target_link_libraries(get_sensor_data ${WIRINGPI_LIB} ${WIRINGPI_DEV_LIB})

# ----------------- Install -----------------
install(TARGETS
  get_sensor_data
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
