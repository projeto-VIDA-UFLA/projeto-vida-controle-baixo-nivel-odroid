#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <iostream>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "../../../../../Utils/Serial/Serial.h"


using namespace std;
using namespace rclcpp;

using placeholders::_1;

class speed_control : public Node
{
private:

    int speedPWM;
    int brakePWM;
    int steeringPWM;

    Serial *actuator_serial;
       
    Subscription<std_msgs::msg::Int16>::SharedPtr subscriber_speed;
    Subscription<std_msgs::msg::Int16>::SharedPtr subscriber_brake;
    Subscription<std_msgs::msg::Int16>::SharedPtr subscriber_steering;

    Publisher<std_msgs::msg::Int16>::SharedPtr speedPublisher;

    void callback_speed(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_brake(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_steering(const std_msgs::msg::Int16::SharedPtr msg);
    

public:
    speed_control(string name);
    ~speed_control();
};

#endif