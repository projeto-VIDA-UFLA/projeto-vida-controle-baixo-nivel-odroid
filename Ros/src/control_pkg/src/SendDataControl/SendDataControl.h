#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "../../../../../Utils/Serial/Serial.h"

using namespace std;
using namespace rclcpp;

using placeholders::_1;

class SendDataControl : public Node
{
private:
    
    Serial *serial;

    double frequencyPublish;

    int steering;
    int speed;
    int brake;

    Subscription<std_msgs::msg::Int16>::SharedPtr steeringSubscription;
    Subscription<std_msgs::msg::Int16>::SharedPtr speedSubSubscription;
    Subscription<std_msgs::msg::Int16>::SharedPtr brakeSubSubscription;

    TimerBase::SharedPtr timer;


    void callbackSteering(const std_msgs::msg::Int16::SharedPtr msgSteering);
    void callbackSpeed(const std_msgs::msg::Int16::SharedPtr msgSpeed);
    void callbackBrake(const std_msgs::msg::Int16::SharedPtr msgBrake);
    void sendMensage();

    int calculateCheckSum(const string& dado);
    

public:
    SendDataControl(string name);
    ~SendDataControl();
};

#endif