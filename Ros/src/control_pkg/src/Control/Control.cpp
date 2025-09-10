#include "Control.h"

Control::Control(string name) : Node(name)
{
    subscriber = this->create_subscription<vida_interfaces::msg::Controls>("/setpoint/speed", 10, bind(&Control::callbackControls, this, _1));
    RCLCPP_INFO(this->get_logger(), "control speed has been started");
}

Control::~Control()
{
    delete motorSerial;
}

void Control::callbackControls(const vida_interfaces::msg::Controls::SharedPtr msg)
{
    string data = to_string(msg->x) + " " + to_string(msg->y);    
    motorSerial->write(data);
}

int main(int argc, char **argv)
{
    init(argc, argv);                            // inicializado o ROS
    auto node = make_shared<Control>("control"); // criação do nó usando ponteiro compartilhado e o nomeando
    spin(node);                                  // gira o nó (o mantem vivo até ctrl + c)
    shutdown();                                  // desliga o nó
    return 0;
}