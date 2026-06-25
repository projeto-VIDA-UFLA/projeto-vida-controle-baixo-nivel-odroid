#ifndef send_data_control_H
#define send_data_control_H

#include <iostream>
#include <string.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "../../../../../Utils/Serial/Serial.h"

using placeholders::_1;
using namespace std;
using namespace rclcpp;

// Nó responsável por receber comandos de PWM via tópicos ROS2
// e enviá-los por comunicação serial ao microcontrolador.
class send_data_control : public Node
{
private:
    // Valores PWM
    int speedPWM;
    int brakePWM;
    int steeringPWM;

    // Últimos tempos de atualização de cada tópico
    Time lastSpeedTime;
    Time lastBrakeTime;
    Time lastSteeringTime;

    // Comunicação serial com os atuadores
    Serial *actuator_serial;

    // Timer para envio periódico dos dados
    TimerBase::SharedPtr timer;

    // Assinaturas dos tópicos
    Subscription<std_msgs::msg::Int16>::SharedPtr subscriber_speed;
    Subscription<std_msgs::msg::Int16>::SharedPtr subscriber_brake;
    Subscription<std_msgs::msg::Int16>::SharedPtr subscriber_steering;

    
    // Callbacks dos tópicos
    void callback_speed(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_brake(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_steering(const std_msgs::msg::Int16::SharedPtr msg);

    // Envio dos dados via serial
    void sendMensage();

    // Cálculo do checksum (XOR simples)
    int calculateCheckSum(const string &dado);

public:
    // Construtor e destrutor
    send_data_control(string name);
    ~send_data_control();
};

#endif
