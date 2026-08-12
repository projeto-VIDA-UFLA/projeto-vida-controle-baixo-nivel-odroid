#include <functional>
#include <memory>

#include "security.h"

Security::Security(const std::string &node_name)
    : Node(node_name, "/vida")
{
  gamepad_enable_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/gamepad/enabled", 10, std::bind(&Security::gamepad_enable_callback, this, std::placeholders::_1));

  logical_enable_publisher_ = this->create_publisher<vida_interfaces::msg::LogicalEnable>("/vida/safety/logical_enable", 10);

  RCLCPP_INFO(this->get_logger(), "Monitorando /vida/gamepad/enabled e publicando /vida/safety/logical_enable");
}

void Security::gamepad_enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  vida_interfaces::msg::LogicalEnable logical_enable;
  logical_enable.timestamp = this->get_clock()->now();
  logical_enable.enabled = msg->data;

  logical_enable_publisher_->publish(logical_enable);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Security>("security"));
  rclcpp::shutdown();
  return 0;
}
