#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include <wiringpi2/wiringPi.h>

#include "../../../../../Utils/Serial/Serial.h"


using namespace std;
using namespace rclcpp;

using placeholders::_1;

class Control : public Node
{
private:
    
    Serial *motorSerial;
    Subscription<vida_interfaces::msg::Controls>::SharedPtr subscriber;

    void callbackControls(const vida_interfaces::msg::Controls::SharedPtr msg);
    

public:
    Control(string name);
    ~Control();
};

#endif