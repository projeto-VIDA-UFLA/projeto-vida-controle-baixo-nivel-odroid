cmake_minimum_required(VERSION 3.15)
project(control_pkg)

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
add_executable(send_data_control
    src/send_data_control/send_data_control.cpp
    src/send_data_control/send_data_control.h
    ../../../Utils/Serial/Serial.cpp
    ../../../Utils/Serial/Serial.h
    ../../../Utils/Files/DefinePin.h
)
ament_target_dependencies(send_data_control rclcpp std_msgs)
target_link_libraries(send_data_control ${WIRINGPI_LIB} ${WIRINGPI_DEV_LIB})

# ----------------- Install -----------------
install(TARGETS
  send_data_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
