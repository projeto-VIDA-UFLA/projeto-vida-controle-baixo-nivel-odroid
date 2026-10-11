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

#include "vida_ntrip_client/ntrip_client_node.hpp"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "vida_ntrip_client/ntrip_protocol.hpp"

namespace vida_ntrip_client
{
namespace
{

bool invalidTextParameter(const std::string & value)
{
  return value.empty() || value.find('\r') != std::string::npos ||
         value.find('\n') != std::string::npos;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string boolText(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

NtripClientNode::NtripClientNode(const rclcpp::NodeOptions & options)
: Node("ntrip_client_node", options)
{
  declareParameters();
  config_ = readAndValidateConfig();
  rtcm_topic_ = get_parameter("rtcm_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();
  fix_topic_ = get_parameter("fix_topic").as_string();

  const char * username_value = std::getenv("VIDA_NTRIP_USERNAME");
  const char * password_value = std::getenv("VIDA_NTRIP_PASSWORD");
  if (username_value == nullptr || username_value[0] == '\0' ||
    password_value == nullptr || password_value[0] == '\0')
  {
    throw std::runtime_error(
            "NTRIP credentials are required in VIDA_NTRIP_USERNAME and VIDA_NTRIP_PASSWORD");
  }

  const auto rtcm_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  rtcm_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>(rtcm_topic_, rtcm_qos);
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, rclcpp::QoS(10));

  if (config_.send_gga) {
    fix_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NtripClientNode::receiveFix, this, std::placeholders::_1));
  }

  connection_ = std::make_unique<NtripConnection>(
    config_, std::string(username_value), std::string(password_value),
    std::bind(&NtripClientNode::publishRtcm, this, std::placeholders::_1),
    std::bind(&NtripClientNode::currentGga, this));
  diagnostics_timer_ = create_wall_timer(
    std::chrono::seconds(1), std::bind(&NtripClientNode::publishDiagnostics, this));
  connection_->start();

  RCLCPP_INFO(
    get_logger(), "NTRIP client configured for %s:%u/%s; RTCM topic: %s",
    config_.caster_host.c_str(), static_cast<unsigned int>(config_.caster_port),
    config_.mountpoint.c_str(),
    rtcm_topic_.c_str());
}

NtripClientNode::~NtripClientNode()
{
  if (connection_) {
    connection_->stop();
  }
}

void NtripClientNode::declareParameters()
{
  declare_parameter<std::string>("caster_host", "170.84.40.52");
  declare_parameter<int64_t>("caster_port", 2101);
  declare_parameter<std::string>("mountpoint", "MGLA0");
  declare_parameter<std::string>("rtcm_topic", "/vida/sensors/gps/rtcm");
  declare_parameter<std::string>(
    "diagnostics_topic", "/vida/sensors/gps/ntrip/diagnostics");
  declare_parameter<std::string>("user_agent", "NTRIP VIDA/1.0");
  declare_parameter<int64_t>("connect_timeout_ms", 5000);
  declare_parameter<int64_t>("header_timeout_ms", 5000);
  declare_parameter<int64_t>("read_timeout_ms", 15000);
  declare_parameter<int64_t>("reconnect_initial_delay_ms", 1000);
  declare_parameter<int64_t>("reconnect_max_delay_ms", 30000);
  declare_parameter<int64_t>("max_response_header_bytes", 8192);
  declare_parameter<int64_t>("publish_chunk_max_bytes", 4096);
  declare_parameter<bool>("enable_rtcm_validation", true);
  declare_parameter<bool>("send_gga", false);
  declare_parameter<std::string>("fix_topic", "/vida/sensors/gps/fix");
  declare_parameter<double>("gga_interval_s", 10.0);
}

NtripConnectionConfig NtripClientNode::readAndValidateConfig()
{
  NtripConnectionConfig config;
  config.caster_host = get_parameter("caster_host").as_string();
  const int64_t port = get_parameter("caster_port").as_int();
  config.mountpoint = get_parameter("mountpoint").as_string();
  config.user_agent = get_parameter("user_agent").as_string();
  const int64_t connect_timeout = get_parameter("connect_timeout_ms").as_int();
  const int64_t header_timeout = get_parameter("header_timeout_ms").as_int();
  const int64_t read_timeout = get_parameter("read_timeout_ms").as_int();
  const int64_t initial_delay = get_parameter("reconnect_initial_delay_ms").as_int();
  const int64_t max_delay = get_parameter("reconnect_max_delay_ms").as_int();
  const int64_t max_header = get_parameter("max_response_header_bytes").as_int();
  const int64_t max_chunk = get_parameter("publish_chunk_max_bytes").as_int();
  config.enable_rtcm_validation = get_parameter("enable_rtcm_validation").as_bool();
  config.send_gga = get_parameter("send_gga").as_bool();
  config.gga_interval_s = get_parameter("gga_interval_s").as_double();

  const std::string rtcm_topic = get_parameter("rtcm_topic").as_string();
  const std::string diagnostics_topic = get_parameter("diagnostics_topic").as_string();
  const std::string fix_topic = get_parameter("fix_topic").as_string();
  if (invalidTextParameter(config.caster_host) || invalidTextParameter(config.mountpoint) ||
    invalidTextParameter(config.user_agent) || invalidTextParameter(rtcm_topic) ||
    invalidTextParameter(diagnostics_topic) || invalidTextParameter(fix_topic))
  {
    throw std::invalid_argument(
            "NTRIP text parameters must be non-empty and contain no line breaks");
  }
  if (port < 1 || port > std::numeric_limits<uint16_t>::max()) {
    throw std::invalid_argument("caster_port must be between 1 and 65535");
  }
  if (connect_timeout <= 0 || header_timeout <= 0 || read_timeout <= 0) {
    throw std::invalid_argument("NTRIP timeout parameters must be positive");
  }
  if (connect_timeout > std::numeric_limits<int>::max() ||
    header_timeout > std::numeric_limits<int>::max() ||
    read_timeout > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument("NTRIP timeout parameters exceed the supported range");
  }
  if (initial_delay <= 0 || max_delay < initial_delay) {
    throw std::invalid_argument(
            "reconnect delays must be positive and maximum must not be below initial");
  }
  if (max_delay > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("NTRIP reconnect delay exceeds the supported range");
  }
  if (max_header < 64 || max_header > 1024 * 1024) {
    throw std::invalid_argument("max_response_header_bytes must be between 64 and 1048576");
  }
  if (max_chunk < 1 || max_chunk > 1024 * 1024) {
    throw std::invalid_argument("publish_chunk_max_bytes must be between 1 and 1048576");
  }
  if (!std::isfinite(config.gga_interval_s) || config.gga_interval_s <= 0.0) {
    throw std::invalid_argument("gga_interval_s must be finite and positive");
  }

  config.caster_port = static_cast<uint16_t>(port);
  config.connect_timeout_ms = static_cast<int>(connect_timeout);
  config.header_timeout_ms = static_cast<int>(header_timeout);
  config.read_timeout_ms = static_cast<int>(read_timeout);
  config.reconnect_initial_delay_ms = static_cast<int>(initial_delay);
  config.reconnect_max_delay_ms = static_cast<int>(max_delay);
  config.max_response_header_bytes = static_cast<std::size_t>(max_header);
  config.publish_chunk_max_bytes = static_cast<std::size_t>(max_chunk);
  return config;
}

void NtripClientNode::publishRtcm(const std::vector<uint8_t> & bytes)
{
  std_msgs::msg::UInt8MultiArray message;
  message.data = bytes;
  rtcm_publisher_->publish(std::move(message));
}

void NtripClientNode::receiveFix(sensor_msgs::msg::NavSatFix::ConstSharedPtr message)
{
  if (message->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
    !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
    message->latitude < -90.0 || message->latitude > 90.0 ||
    message->longitude < -180.0 || message->longitude > 180.0)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(fix_mutex_);
  last_fix_ = std::make_pair(message->latitude, message->longitude);
}

std::optional<std::string> NtripClientNode::currentGga() const
{
  std::pair<double, double> fix;
  {
    std::lock_guard<std::mutex> lock(fix_mutex_);
    if (!last_fix_.has_value()) {
      return std::nullopt;
    }
    fix = last_fix_.value();
  }

  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  if (::gmtime_r(&now, &utc) == nullptr) {
    return std::nullopt;
  }
  std::ostringstream timestamp;
  timestamp << std::put_time(&utc, "%H%M%S");
  return buildGgaSentence(fix.first, fix.second, timestamp.str());
}

void NtripClientNode::publishDiagnostics()
{
  const ConnectionSnapshot snapshot = connection_->snapshot();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "vida_ntrip_client/caster";
  status.hardware_id = config_.caster_host + ":" + std::to_string(config_.caster_port);
  status.message = networkStateName(snapshot.state);
  if (snapshot.state == NetworkState::kStreaming) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else if (snapshot.state == NetworkState::kBackoff || !snapshot.last_error.empty()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  }

  status.values.push_back(keyValue("state", networkStateName(snapshot.state)));
  status.values.push_back(keyValue("host", config_.caster_host));
  status.values.push_back(keyValue("port", std::to_string(config_.caster_port)));
  status.values.push_back(keyValue("mountpoint", config_.mountpoint));
  status.values.push_back(keyValue("connected", boolText(snapshot.connected)));
  status.values.push_back(keyValue("last_byte_age_s", std::to_string(snapshot.last_byte_age_s)));
  status.values.push_back(keyValue("bytes_received", std::to_string(snapshot.bytes_received)));
  status.values.push_back(keyValue("bytes_published", std::to_string(snapshot.bytes_published)));
  status.values.push_back(keyValue("connections", std::to_string(snapshot.connections)));
  status.values.push_back(keyValue("reconnections", std::to_string(snapshot.reconnections)));
  status.values.push_back(keyValue("last_error", snapshot.last_error));
  status.values.push_back(
    keyValue(
      "rtcm_valid_frames",
      std::to_string(snapshot.rtcm.valid_frames)));
  status.values.push_back(
    keyValue(
      "rtcm_invalid_crc_frames", std::to_string(snapshot.rtcm.invalid_crc_frames)));

  constexpr uint16_t kObservedTypes[] = {
    1006, 1008, 1013, 1019, 1020, 1033, 1074, 1084, 1094, 1124, 1230};
  for (const uint16_t type : kObservedTypes) {
    const auto count = snapshot.rtcm.message_type_counts.find(type);
    status.values.push_back(
      keyValue(
        "rtcm_type_" + std::to_string(type),
        std::to_string(count == snapshot.rtcm.message_type_counts.end() ? 0U : count->second)));
  }

  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(std::move(array));
}

}  // namespace vida_ntrip_client
