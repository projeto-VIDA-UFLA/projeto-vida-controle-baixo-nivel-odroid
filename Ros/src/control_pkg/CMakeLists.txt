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


# ----------------- Control -----------------
add_executable(control
    src/Control/Control.cpp
    src/Control/Control.h
    ../../../Utils/Serial/Serial.cpp
    ../../../Utils/Serial/Serial.h
)
ament_target_dependencies(control rclcpp std_msgs)
target_link_libraries(control ${WIRINGPI_LIB} ${WIRINGPI_DEV_LIB})

# ----------------- Sensors -----------------
add_executable(sensors
    src/Sensors/Sensors.cpp
    src/Sensors/Sensors.h
    ../../../Utils/Serial/Serial.cpp
    ../../../Utils/Serial/Serial.h
    ../../../Utils/Files/DefinePin.h
)
ament_target_dependencies(sensors rclcpp vida_interfaces)
target_link_libraries(sensors ${WIRINGPI_LIB} ${WIRINGPI_DEV_LIB})

# ----------------- SendDataControl -----------------
add_executable(send_data_control
    src/SendDataControl/SendDataControl.cpp
    src/SendDataControl/SendDataControl.h
    ../../../Utils/Serial/Serial.cpp
    ../../../Utils/Serial/Serial.h
    ../../../Utils/Files/DefinePin.h
)
ament_target_dependencies(send_data_control rclcpp std_msgs)
target_link_libraries(send_data_control ${WIRINGPI_LIB} ${WIRINGPI_DEV_LIB})

# ----------------- Install -----------------
install(TARGETS
  control 
  sensors 
  send_data_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
