#include "Control.h"

Control::Control(string name) : Node(name)
{
    subscriber = this->create_subscription<vida_interfaces::msg::Controls>("/setpoint/speed", 10, bind(&Control::callbackControls, this, _1));
    speedPublisher = this->create_publisher<std_msgs::msg::Int16>("/pwm/speed", 10);
    RCLCPP_INFO(this->get_logger(), "control speed has been started");
}

Control::~Control()
{
    delete motorSerial;
}

void Control::callbackControls(const std_msgs::msg::Int16::SharedPtr speedValue)
{
    int pwm;

    if (speedValue < 20)
    {
        speedValue = 0;
        pwm = 0;
    }

    else
    {
        pwm = map(speedValue, 20, 512, 64, 125);
    }

    speedPublisher->publish(pwm);
}

int main(int argc, char **argv)
{
    init(argc, argv);                            // inicializado o ROS
    auto node = make_shared<Control>("control"); // criação do nó usando ponteiro compartilhado e o nomeando
    spin(node);                                  // gira o nó (o mantem vivo até ctrl + c)
    shutdown();                                  // desliga o nó
    return 0;
}