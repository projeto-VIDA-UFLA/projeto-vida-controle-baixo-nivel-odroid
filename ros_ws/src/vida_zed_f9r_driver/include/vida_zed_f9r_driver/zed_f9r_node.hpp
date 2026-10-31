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

#ifndef VIDA_ZED_F9R_DRIVER__ZED_F9R_NODE_HPP_
#define VIDA_ZED_F9R_DRIVER__ZED_F9R_NODE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "vida_interfaces/msg/gnss_status.hpp"

#include "vida_zed_f9r_driver/serial_transport.hpp"
#include "vida_zed_f9r_driver/ubx_parser.hpp"
#include "vida_zed_f9r_driver/ubx_protocol.hpp"

namespace vida_zed_f9r_driver
{

class ZedF9rNode : public rclcpp::Node
{
public:
  explicit ZedF9rNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ZedF9rNode() override;

private:
  struct PendingEpoch
  {
    std::optional<NavPvt> pvt;
    std::optional<NavHpPosLlh> high_precision_position;
    builtin_interfaces::msg::Time reception_stamp;
    bool has_stamp{false};
    bool status_published{false};
  };

  struct ImuSample
  {
    std::array<double, 3> angular_velocity{};
    std::array<double, 3> linear_acceleration{};
    std::array<bool, 3> has_angular_velocity{};
    std::array<bool, 3> has_linear_acceleration{};

    bool complete() const;
  };

  void declareAndReadParameters();
  void validateAxisMapping(const std::vector<int64_t> & mapping);
  void onTransportState(bool connected, const std::string & detail);
  void onBytes(const uint8_t * data, std::size_t size);
  void handleFrame(const UbxFrame & frame);
  void handleAcknowledgement(const UbxFrame & frame, bool acknowledged);
  void handleNavPvt(const std::vector<uint8_t> & payload);
  void handleNavHpPosLlh(const std::vector<uint8_t> & payload);
  void handleNavCov(const std::vector<uint8_t> & payload);
  void handleEsfRaw(const std::vector<uint8_t> & payload);
  void handleMonVersion(const std::vector<uint8_t> & payload);
  void handleRxmRtcm(const std::vector<uint8_t> & payload);
  void publishGnssStatus(PendingEpoch & epoch);
  void publishGnssFixIfComplete(uint32_t i_tow_ms);
  void publishImuSample(const ImuSample & sample);
  void onRtcm(const std_msgs::msg::UInt8MultiArray::SharedPtr message);
  void sendStartupRequests();
  void checkConfigurationAcknowledgement();
  void logStatistics();
  bool correctionsRecentlyReceived() const;
  void prunePendingData();
  std::array<double, 3> mapAxes(const std::array<double, 3> & native) const;

  std::string device_;
  std::string frame_id_;
  int baudrate_{115200};
  int reconnect_interval_ms_{1000};
  int configuration_ack_timeout_ms_{2000};
  int priority_navigation_rate_hz_{0};
  int measurement_rate_ms_{1000};
  double correction_received_timeout_s_{10.0};
  bool disable_internal_sensor_fusion_{true};
  bool apply_receiver_configuration_{true};
  bool publish_raw_imu_{true};
  bool publish_gnss_{true};
  bool enable_rtcm_diagnostics_{true};
  std::array<int, 3> imu_axis_mapping_{{1, 2, 3}};

  UbxParser parser_;
  std::unique_ptr<SerialTransport> transport_;
  std::unordered_map<uint32_t, PendingEpoch> pending_epochs_;
  std::unordered_map<uint32_t, NavCov> covariances_;
  std::unordered_map<uint32_t, ImuSample> imu_samples_;
  std::optional<std::chrono::steady_clock::time_point> last_valid_rtcm_;

  std::mutex configuration_mutex_;
  bool configuration_ack_pending_{false};
  std::chrono::steady_clock::time_point configuration_ack_deadline_;

  std::atomic<uint64_t> rtcm_messages_sent_{0};
  std::atomic<uint64_t> rtcm_bytes_sent_{0};
  std::atomic<uint64_t> rtcm_receiver_messages_{0};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  rclcpp::Publisher<vida_interfaces::msg::GnssStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rtcm_subscription_;
  rclcpp::TimerBase::SharedPtr acknowledgement_timer_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
};

}  // namespace vida_zed_f9r_driver

#endif  // VIDA_ZED_F9R_DRIVER__ZED_F9R_NODE_HPP_
