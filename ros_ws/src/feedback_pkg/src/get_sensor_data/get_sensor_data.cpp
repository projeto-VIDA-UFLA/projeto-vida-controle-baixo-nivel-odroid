#include "get_sensor_data.h"

get_sensor_data::get_sensor_data(string name) : Node(name)
{
    this->declare_parameter("frequency_publish", 20.0);
    float frequencyPublish = this->get_parameter("frequency_publish").as_double();

    sensorSerial = new Serial("ttyACM0");

    wiringPiSetup();
    pinMode(sensorInterrupt, OUTPUT);
    digitalWrite(sensorInterrupt, LOW);

    publisher_speed = this->create_publisher<std_msgs::msg::Float64>("/current_state/speed", 10);
    publisher_steering = this->create_publisher<std_msgs::msg::Float64>("/current_state/steering", 10);

    timer = this->create_wall_timer(chrono::nanoseconds((int)(1e9 / frequencyPublish)), bind(&get_sensor_data::readSensors, this));
    RCLCPP_INFO(this->get_logger(), "sensors has been started");
}

get_sensor_data::~get_sensor_data()
{
    delete sensorSerial;
}

void get_sensor_data::readSensors()
{
    digitalWrite(sensorInterrupt, HIGH);
    dataRecived = sensorSerial->read('\n');
    digitalWrite(sensorInterrupt, LOW);

    int left_pulses;
    int right_pulses;
    int back_pulses;
    int steering_angle;
    

    sscanf(dataRecived.c_str(), "%ld %ld %ld %le", left_pulses, right_pulses, back_pulses, steering_angle);

    float linear_velocity = 0; 
    float steering_angle = steering_angle*(-0.0889) + 49.667;

    auto speed = std_msgs::msg::Float64();
    auto steering = std_msgs::msg::Float64();

    speed.data = linear_velocity;
    steering.data = steering_angle; 

    publisher_speed->publish(speed);
    publisher_steering->publish(steering);
}

int main(int argc, char **argv)
{
    init(argc, argv);                                            // inicializado o ROS
    auto node = make_shared<get_sensor_data>("get_sensor_data"); // criação do nó usando ponteiro compartilhado e o nomeando
    spin(node);                                                  // gira o nó (o mantem vivo até ctrl + c)
    shutdown();                                                  // desliga o nó
    return 0;
}