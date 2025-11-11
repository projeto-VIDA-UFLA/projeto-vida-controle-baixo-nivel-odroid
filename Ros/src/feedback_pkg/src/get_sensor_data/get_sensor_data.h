#ifndef get_sensor_data_H
#define get_sensor_data_H

#include <iostream>
#include <string.h>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <wiringpi2/wiringPi.h>
#include "../../../../../Utils/Serial/Serial.h"
#include "../../../../../Utils/Files/DefinePin.h"

using namespace std;
using namespace rclcpp;

class get_sensor_data : public Node
{
private:
    
    string dataRecived;

    Serial *sensorSerial;
    TimerBase::SharedPtr timer;
    
    Publisher<std_msgs::msg::Float64>::SharedPtr publisher_speed;
    Publisher<std_msgs::msg::Float64>::SharedPtr publisher_steering;    
    
    void readSensors();

public:
    get_sensor_data(string name);
    ~get_sensor_data();
};

#endif