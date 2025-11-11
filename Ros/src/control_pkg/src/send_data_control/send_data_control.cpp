#include "send_data_control.h"

send_data_control::send_data_control(string name) : Node(name)
{
    // Inicializa comunicação serial
    actuator_serial = new Serial("ttyS1");

    // Inicializa variáveis PWM
    speedPWM = 0;
    brakePWM = 0;
    steeringPWM = 0;

    // Marca o tempo inicial de cada tópico
    lastSpeedTime = this->get_clock()->now();
    lastBrakeTime = this->get_clock()->now();
    lastSteeringTime = this->get_clock()->now();

    // Frequência de envio (Hz)
    this->declare_parameter("frequency_publish", 100.0);
    float frequencyPublish = this->get_parameter("frequency_publish").as_double();

    // Assinaturas dos tópicos PWM
    subscriber_speed = this->create_subscription<std_msgs::msg::Int16>("/pwm/speed", 10, bind(&send_data_control::callback_speed, this, _1));
    subscriber_brake = this->create_subscription<std_msgs::msg::Int16>("/pwm/brake", 10, bind(&send_data_control::callback_brake, this, _1));
    subscriber_steering = this->create_subscription<std_msgs::msg::Int16>("/pwm/steering", 10, bind(&send_data_control::callback_steering, this, _1));

    // Timer periódico para envio serial
    timer = this->create_wall_timer(chrono::nanoseconds((int)(1e9 / frequencyPublish)),bind(&send_data_control::sendMensage, this));

    RCLCPP_INFO(this->get_logger(), "send_data_control iniciado");
}

send_data_control::~send_data_control()
{
    delete actuator_serial;
}

// Callback: atualiza PWM da velocidade
void send_data_control::callback_speed(const std_msgs::msg::Int16::SharedPtr msg)
{
    lastSpeedTime = this->get_clock()->now();
    speedPWM = msg->data;
}

// Callback: atualiza PWM do freio
void send_data_control::callback_brake(const std_msgs::msg::Int16::SharedPtr msg)
{
    lastBrakeTime = this->get_clock()->now();
    brakePWM = msg->data;
}

// Callback: atualiza PWM da direção
void send_data_control::callback_steering(const std_msgs::msg::Int16::SharedPtr msg)
{
    lastSteeringTime = this->get_clock()->now();
    steeringPWM = msg->data;
}

// Envia dados via serial com verificação de timeout
void send_data_control::sendMensage()
{
    Time now = this->get_clock()->now();

    Duration diffSpeed = now - lastSpeedTime;
    Duration diffBrake = now - lastBrakeTime;
    Duration diffSteering = now - lastSteeringTime;

    // Verifica perda de comunicação
    if (diffSpeed.seconds() > 0.1 && speedPWM != 0)
    {
        RCLCPP_WARN(this->get_logger(), "Sem comunicação com velocidade!");
        speedPWM = 0;
    }

    if (diffBrake.seconds() > 0.1 && brakePWM != 0)
    {
        RCLCPP_WARN(this->get_logger(), "Sem comunicação com freio!");
        brakePWM = 0;
    }

    if (diffSteering.seconds() > 0.1 && steeringPWM != 0)
    {
        RCLCPP_WARN(this->get_logger(), "Sem comunicação com esterçamento!");
        steeringPWM = 0;
    }

    // Monta mensagem com checksum
    string dado = to_string(steeringPWM) + "," + to_string(speedPWM) + "," + to_string(brakePWM);
    int checkSum = calculateCheckSum(dado);

    stringstream ss;
    ss << "#" << dado << "*" << setw(2) << setfill('0') << hex << uppercase << checkSum;

    string msg = ss.str();
    RCLCPP_INFO(this->get_logger(), msg.c_str());

    actuator_serial->write(msg);
}

// Calcula checksum (XOR simples)
int send_data_control::calculateCheckSum(const string &dado)
{
    int calculado = 0;
    for (int i = 0; i < dado.length(); i++)
        calculado ^= dado[i];
    return calculado;
}

// Função principal do nó
int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = make_shared<send_data_control>("send_data_control");
    spin(node);
    shutdown();
    return 0;
}
