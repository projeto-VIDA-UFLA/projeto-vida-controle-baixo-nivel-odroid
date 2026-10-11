// Copyright 2026 Projeto VIDA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VIDA_NTRIP_CLIENT__NTRIP_CLIENT_NODE_HPP_
#define VIDA_NTRIP_CLIENT__NTRIP_CLIENT_NODE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "vida_ntrip_client/ntrip_connection.hpp"

namespace vida_ntrip_client
{

class NtripClientNode : public rclcpp::Node
{
public:
  explicit NtripClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NtripClientNode() override;

private:
  void declareParameters();
  NtripConnectionConfig readAndValidateConfig();
  void publishRtcm(const std::vector<uint8_t> & bytes);
  void publishDiagnostics();
  void receiveFix(sensor_msgs::msg::NavSatFix::ConstSharedPtr message);
  std::optional<std::string> currentGga() const;

  std::string diagnostics_topic_;
  std::string fix_topic_;
  std::string rtcm_topic_;
  NtripConnectionConfig config_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rtcm_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  mutable std::mutex fix_mutex_;
  std::optional<std::pair<double, double>> last_fix_;
  std::unique_ptr<NtripConnection> connection_;
};

}  // namespace vida_ntrip_client

#endif  // VIDA_NTRIP_CLIENT__NTRIP_CLIENT_NODE_HPP_
