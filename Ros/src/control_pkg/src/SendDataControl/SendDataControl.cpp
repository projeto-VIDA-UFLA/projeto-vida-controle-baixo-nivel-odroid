#include "SendDataControl.h"

SendDataControl::SendDataControl(string name) : Node(name)
{
    serial = new Serial("ttyS1");

    steering = 0;
    speed = 0;
    brake = 0;

    this->declare_parameter("frequency_publish", 100.0);
    frequencyPublish = this->get_parameter("frequency_publish").as_double();

    steeringSubscription = this->create_subscription<std_msgs::msg::Int16>("steering", 10, bind(&SendDataControl::callbackSteering, this, _1));
    speedSubSubscription = this->create_subscription<std_msgs::msg::Int16>("speed", 10, bind(&SendDataControl::callbackSpeed, this, _1));
    brakeSubSubscription = this->create_subscription<std_msgs::msg::Int16>("brake", 10, bind(&SendDataControl::callbackBrake, this, _1));
    timer = this->create_wall_timer(chrono::nanoseconds((int)(1e9 / frequencyPublish)), bind(&SendDataControl::sendMensage, this));

    RCLCPP_INFO(this->get_logger(), "control has been started");
}

SendDataControl::~SendDataControl()
{
    delete serial;
}

void SendDataControl::callbackSteering(const std_msgs::msg::Int16::SharedPtr msg)
{
    steering = msg->data;
}

void SendDataControl::callbackSpeed(const std_msgs::msg::Int16::SharedPtr msg)
{
    speed = msg->data;
}

void SendDataControl::callbackBrake(const std_msgs::msg::Int16::SharedPtr msg)
{
    brake = msg->data;
}

void SendDataControl::sendMensage()
{
    string dado = to_string(steering) + "," + to_string(speed) + "," + to_string(brake);
    int checkSum = calculateCheckSum(dado);

    stringstream ss;
    ss << "#" << dado << "*" << hex << uppercase << checkSum;

    string msg = ss.str();
    serial->write(msg);
}

int SendDataControl::calculateCheckSum(const string &dado)
{
    int calculado = 0;
    for (int i = 0; i < dado.length(); i++)
    {
        calculado ^= dado[i];
    }
    return calculado;
}

int main(int argc, char **argv)
{
    init(argc, argv);                                              // inicializado o ROS
    auto node = make_shared<SendDataControl>("send_data_control"); // criação do nó usando ponteiro compartilhado e o nomeando
    spin(node);                                                    // gira o nó (o mantem vivo até ctrl + c)
    shutdown();                                                    // desliga o nó
    return 0;
}