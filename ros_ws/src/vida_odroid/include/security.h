#ifndef VIDA_ODROID__SECURITY_H_
#define VIDA_ODROID__SECURITY_H_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vida_interfaces/msg/logical_enable.hpp"

// Converte /vida/gamepad/enabled no enable logico do veiculo.
class Security : public rclcpp::Node
{
public:
  explicit Security(const std::string & node_name);

private:
  void gamepad_enable_callback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gamepad_enable_subscription_;
  rclcpp::Publisher<vida_interfaces::msg::LogicalEnable>::SharedPtr logical_enable_publisher_;
};

#endif  // VIDA_ODROID__SECURITY_H_
