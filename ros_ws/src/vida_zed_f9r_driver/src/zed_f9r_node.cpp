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

#include "vida_zed_f9r_driver/zed_f9r_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace vida_zed_f9r_driver
{

bool ZedF9rNode::ImuSample::complete() const
{
  return std::all_of(
    has_angular_velocity.begin(), has_angular_velocity.end(), [](bool value) {return value;}) &&
         std::all_of(
    has_linear_acceleration.begin(), has_linear_acceleration.end(), [](bool value) {return value;});
}

ZedF9rNode::ZedF9rNode(const rclcpp::NodeOptions & options)
: Node("zed_f9r_driver", options), parser_(8192U)
{
  declareAndReadParameters();

  const std::string fix_topic = get_parameter("gps_fix_topic").as_string();
  const std::string status_topic = get_parameter("gps_status_topic").as_string();
  const std::string imu_topic = get_parameter("imu_topic").as_string();
  const std::string rtcm_topic = get_parameter("rtcm_topic").as_string();

  if (publish_gnss_) {
    fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      fix_topic, rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<vida_interfaces::msg::GnssStatus>(
      status_topic, rclcpp::QoS(10));
  }
  if (publish_raw_imu_) {
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS());
  }
  rtcm_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    rtcm_topic, rclcpp::QoS(10),
    std::bind(&ZedF9rNode::onRtcm, this, std::placeholders::_1));

  acknowledgement_timer_ = create_wall_timer(
    std::chrono::milliseconds(250),
    std::bind(&ZedF9rNode::checkConfigurationAcknowledgement, this));
  statistics_timer_ = create_wall_timer(
    std::chrono::seconds(30), std::bind(&ZedF9rNode::logStatistics, this));

  transport_ = std::make_unique<SerialTransport>(
    device_, baudrate_, reconnect_interval_ms_,
    std::bind(&ZedF9rNode::onBytes, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ZedF9rNode::onTransportState, this, std::placeholders::_1, std::placeholders::_2));
  transport_->start();

  if (!apply_receiver_configuration_) {
    RCLCPP_WARN(
      get_logger(),
      "apply_receiver_configuration=false; fusao interna e navegacao prioritaria "
      "nao serao configuradas");
  }
  RCLCPP_INFO(
    get_logger(), "ZED-F9R: aguardando %s; frame=%s; GNSS=%s; IMU=%s",
    device_.c_str(), frame_id_.c_str(), publish_gnss_ ? "on" : "off",
    publish_raw_imu_ ? "on" : "off");
}

ZedF9rNode::~ZedF9rNode()
{
  if (transport_) {
    transport_->stop();
  }
}

void ZedF9rNode::declareAndReadParameters()
{
  declare_parameter<std::string>("device", "/dev/vida_f9r");
  declare_parameter<std::string>("frame_id", "f9r_link");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<int>("reconnect_interval_ms", 1000);
  declare_parameter<int>("configuration_ack_timeout_ms", 2000);
  declare_parameter<int>("priority_navigation_rate_hz", 0);
  declare_parameter<int>("measurement_rate_ms", 1000);
  declare_parameter<double>("correction_received_timeout_s", 10.0);
  declare_parameter<bool>("disable_internal_sensor_fusion", true);
  declare_parameter<bool>("apply_receiver_configuration", true);
  declare_parameter<bool>("publish_raw_imu", true);
  declare_parameter<bool>("publish_gnss", true);
  declare_parameter<bool>("enable_rtcm_diagnostics", true);
  declare_parameter<std::vector<int64_t>>("imu_axis_mapping", {1, 2, 3});
  declare_parameter<std::string>("gps_fix_topic", "/vida/sensors/gps/fix");
  declare_parameter<std::string>("gps_status_topic", "/vida/sensors/gps/status");
  declare_parameter<std::string>("imu_topic", "/vida/sensors/imu/data_raw");
  declare_parameter<std::string>("rtcm_topic", "/vida/sensors/gps/rtcm");

  device_ = get_parameter("device").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  baudrate_ = static_cast<int>(get_parameter("baudrate").as_int());
  reconnect_interval_ms_ = static_cast<int>(get_parameter("reconnect_interval_ms").as_int());
  configuration_ack_timeout_ms_ = static_cast<int>(
    get_parameter("configuration_ack_timeout_ms").as_int());
  priority_navigation_rate_hz_ = static_cast<int>(
    get_parameter("priority_navigation_rate_hz").as_int());
  measurement_rate_ms_ = static_cast<int>(get_parameter("measurement_rate_ms").as_int());
  correction_received_timeout_s_ = get_parameter("correction_received_timeout_s").as_double();
  disable_internal_sensor_fusion_ = get_parameter("disable_internal_sensor_fusion").as_bool();
  apply_receiver_configuration_ = get_parameter("apply_receiver_configuration").as_bool();
  publish_raw_imu_ = get_parameter("publish_raw_imu").as_bool();
  publish_gnss_ = get_parameter("publish_gnss").as_bool();
  enable_rtcm_diagnostics_ = get_parameter("enable_rtcm_diagnostics").as_bool();
  validateAxisMapping(get_parameter("imu_axis_mapping").as_integer_array());

  if (device_.empty() || frame_id_.empty()) {
    throw std::invalid_argument("device e frame_id nao podem ser vazios");
  }
  if (reconnect_interval_ms_ <= 0 || configuration_ack_timeout_ms_ <= 0 ||
    correction_received_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("intervalos e timeouts devem ser positivos");
  }
  if (priority_navigation_rate_hz_ < 0 || priority_navigation_rate_hz_ > 30) {
    throw std::invalid_argument("priority_navigation_rate_hz deve estar entre 0 e 30 Hz");
  }
  if (measurement_rate_ms_ < 25 || measurement_rate_ms_ > 10000) {
    throw std::invalid_argument("measurement_rate_ms deve estar entre 25 e 10000 ms");
  }
}

void ZedF9rNode::validateAxisMapping(const std::vector<int64_t> & mapping)
{
  if (mapping.size() != 3U) {
    throw std::invalid_argument("imu_axis_mapping deve conter exatamente tres eixos");
  }
  std::array<bool, 3> used{{false, false, false}};
  for (std::size_t index = 0; index < mapping.size(); ++index) {
    const int64_t absolute_axis = std::abs(mapping[index]);
    if (absolute_axis < 1 || absolute_axis > 3 || used[absolute_axis - 1U]) {
      throw std::invalid_argument(
              "imu_axis_mapping deve ser uma permutacao assinada de [1, 2, 3]");
    }
    used[absolute_axis - 1U] = true;
    imu_axis_mapping_[index] = static_cast<int>(mapping[index]);
  }
}

void ZedF9rNode::onTransportState(bool connected, const std::string & detail)
{
  if (connected) {
    parser_.reset();
    RCLCPP_INFO(get_logger(), "Conectado ao receptor em %s", detail.c_str());
    sendStartupRequests();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(configuration_mutex_);
    configuration_ack_pending_ = false;
  }
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 5000, "ZED-F9R desconectado: %s", detail.c_str());
}

void ZedF9rNode::onBytes(const uint8_t * data, std::size_t size)
{
  for (const UbxFrame & frame : parser_.feed(data, size)) {
    handleFrame(frame);
  }
}

void ZedF9rNode::handleFrame(const UbxFrame & frame)
{
  const bool ack_ack = frame.message_class == kClassAck && frame.message_id == kIdAckAck;
  const bool ack_nak = frame.message_class == kClassAck && frame.message_id == kIdAckNak;
  const bool nav_pvt = frame.message_class == kClassNav && frame.message_id == kIdNavPvt;
  const bool nav_hpposllh =
    frame.message_class == kClassNav && frame.message_id == kIdNavHpPosLlh;
  const bool nav_cov = frame.message_class == kClassNav && frame.message_id == kIdNavCov;
  const bool esf_raw = frame.message_class == kClassEsf && frame.message_id == kIdEsfRaw;
  const bool mon_ver = frame.message_class == kClassMon && frame.message_id == kIdMonVer;
  const bool rxm_rtcm = frame.message_class == kClassRxm && frame.message_id == kIdRxmRtcm;

  if (ack_ack) {
    handleAcknowledgement(frame, true);
  } else if (ack_nak) {
    handleAcknowledgement(frame, false);
  } else if (nav_pvt && publish_gnss_) {
    handleNavPvt(frame.payload);
  } else if (nav_hpposllh && publish_gnss_) {
    handleNavHpPosLlh(frame.payload);
  } else if (nav_cov && publish_gnss_) {
    handleNavCov(frame.payload);
  } else if (esf_raw && publish_raw_imu_) {
    handleEsfRaw(frame.payload);
  } else if (mon_ver) {
    handleMonVersion(frame.payload);
  } else if (rxm_rtcm) {
    handleRxmRtcm(frame.payload);
  }
}

void ZedF9rNode::handleAcknowledgement(const UbxFrame & frame, bool acknowledged)
{
  if (frame.payload.size() != 2U || frame.payload[0U] != kClassCfg ||
    frame.payload[1U] != kIdCfgValset)
  {
    return;
  }

  bool was_pending = false;
  {
    std::lock_guard<std::mutex> lock(configuration_mutex_);
    was_pending = configuration_ack_pending_;
    configuration_ack_pending_ = false;
  }
  if (!was_pending) {
    return;
  }
  if (acknowledged) {
    RCLCPP_INFO(
      get_logger(),
      "UBX-CFG-VALSET confirmado (somente RAM); fusao interna=%s; "
      "navegacao prioritaria=%d Hz; medicao a cada %d ms",
      disable_internal_sensor_fusion_ ? "desativada" : "ativada",
      priority_navigation_rate_hz_, measurement_rate_ms_);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Receptor respondeu UBX-ACK-NAK ao UBX-CFG-VALSET; recepcao de dados continua");
  }
}

void ZedF9rNode::handleNavPvt(const std::vector<uint8_t> & payload)
{
  const std::optional<NavPvt> parsed = parseNavPvt(payload);
  if (!parsed.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "UBX-NAV-PVT com payload invalido");
    return;
  }

  PendingEpoch & epoch = pending_epochs_[parsed->i_tow_ms];
  if (!epoch.has_stamp) {
    epoch.reception_stamp = now();
    epoch.has_stamp = true;
  }
  epoch.pvt = parsed;
  publishGnssStatus(epoch);
  if (parsed->fix_type == 4U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "NAV-PVT indica solucao GNSS+DR; a posicao fusionada nao sera publicada como fix valido");
  }
  publishGnssFixIfComplete(parsed->i_tow_ms);
  prunePendingData();
}

void ZedF9rNode::handleNavHpPosLlh(const std::vector<uint8_t> & payload)
{
  const std::optional<NavHpPosLlh> parsed = parseNavHpPosLlh(payload);
  if (!parsed.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "UBX-NAV-HPPOSLLH com payload invalido");
    return;
  }

  PendingEpoch & epoch = pending_epochs_[parsed->i_tow_ms];
  if (!epoch.has_stamp) {
    epoch.reception_stamp = now();
    epoch.has_stamp = true;
  }
  epoch.high_precision_position = parsed;
  publishGnssFixIfComplete(parsed->i_tow_ms);
  prunePendingData();
}

void ZedF9rNode::handleNavCov(const std::vector<uint8_t> & payload)
{
  const std::optional<NavCov> parsed = parseNavCov(payload);
  if (!parsed.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "UBX-NAV-COV com payload invalido");
    return;
  }
  covariances_[parsed->i_tow_ms] = parsed.value();
  prunePendingData();
}

void ZedF9rNode::handleEsfRaw(const std::vector<uint8_t> & payload)
{
  const std::optional<std::vector<EsfRawMeasurement>> measurements = parseEsfRaw(payload);
  if (!measurements.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "UBX-ESF-RAW com payload invalido");
    return;
  }

  for (const EsfRawMeasurement & measurement : measurements.value()) {
    int axis = -1;
    bool gyroscope = false;
    switch (measurement.data_type) {
      case kEsfGyroX:
        axis = 0;
        gyroscope = true;
        break;
      case kEsfGyroY:
        axis = 1;
        gyroscope = true;
        break;
      case kEsfGyroZ:
        axis = 2;
        gyroscope = true;
        break;
      case kEsfAccelX:
        axis = 0;
        break;
      case kEsfAccelY:
        axis = 1;
        break;
      case kEsfAccelZ:
        axis = 2;
        break;
      default:
        continue;
    }

    ImuSample & sample = imu_samples_[measurement.sensor_time_tag];
    if (gyroscope) {
      sample.angular_velocity[static_cast<std::size_t>(axis)] =
        gyroscopeRadiansPerSecond(measurement.raw_value);
      sample.has_angular_velocity[static_cast<std::size_t>(axis)] = true;
    } else {
      sample.linear_acceleration[static_cast<std::size_t>(axis)] =
        accelerationMetersPerSecondSquared(measurement.raw_value);
      sample.has_linear_acceleration[static_cast<std::size_t>(axis)] = true;
    }
    if (sample.complete()) {
      publishImuSample(sample);
      imu_samples_.erase(measurement.sensor_time_tag);
    }
  }
  prunePendingData();
}

void ZedF9rNode::handleMonVersion(const std::vector<uint8_t> & payload)
{
  const std::optional<MonVersion> version = parseMonVersion(payload);
  if (!version.has_value()) {
    RCLCPP_WARN(get_logger(), "UBX-MON-VER com payload invalido");
    return;
  }
  const std::string firmware = version->extensionValue("FWVER=");
  const std::string protocol = version->extensionValue("PROTVER=");
  const std::string module = version->extensionValue("MOD=");
  RCLCPP_INFO(
    get_logger(), "u-blox MON-VER: SW='%s' HW='%s' FWVER='%s' PROTVER='%s' MOD='%s'",
    version->software.c_str(), version->hardware.c_str(), firmware.c_str(), protocol.c_str(),
    module.c_str());

  if (protocol != "33.30") {
    RCLCPP_WARN(
      get_logger(), "Protocolo '%s' difere do validado (33.30)", protocol.c_str());
  }
  if (module.find("ZED-F9R") == std::string::npos) {
    RCLCPP_WARN(get_logger(), "Modulo '%s' difere do validado (ZED-F9R)", module.c_str());
  }
  if (firmware.find("HPS 1.30") == std::string::npos) {
    RCLCPP_WARN(get_logger(), "Firmware '%s' difere do validado (HPS 1.30)", firmware.c_str());
  }
}

void ZedF9rNode::handleRxmRtcm(const std::vector<uint8_t> & payload)
{
  const std::optional<RxmRtcm> rtcm = parseRxmRtcm(payload);
  if (!rtcm.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "UBX-RXM-RTCM com payload invalido");
    return;
  }
  if (rtcm->crc_failed) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Receptor informou RTCM com CRC invalido");
    return;
  }
  last_valid_rtcm_ = std::chrono::steady_clock::now();
  ++rtcm_receiver_messages_;
}

void ZedF9rNode::publishGnssStatus(PendingEpoch & epoch)
{
  if (epoch.status_published || !epoch.pvt.has_value() || !status_publisher_) {
    return;
  }
  const NavPvt & pvt = epoch.pvt.value();
  vida_interfaces::msg::GnssStatus message;
  message.timestamp = epoch.reception_stamp;
  message.fix_type = pvt.fix_type;
  const uint8_t carrier_solution = pvt.carrierSolution();
  message.rtk_status = carrier_solution <= 2U ? carrier_solution : 0U;
  message.satellites = pvt.num_sv;
  message.gnss_fix_ok = pvt.gnssFixOk();
  message.differential_solution = pvt.differentialSolution();
  message.corrections_received = pvt.correctionAgeCode() != 0U || correctionsRecentlyReceived();
  message.horizontal_accuracy_m = static_cast<float>(pvt.horizontal_accuracy_mm) * 1.0e-3F;
  message.vertical_accuracy_m = static_cast<float>(pvt.vertical_accuracy_mm) * 1.0e-3F;
  message.correction_age_s = correctionAgeLowerBoundSeconds(pvt.correctionAgeCode());
  if (pvt.correctionAgeCode() == 0U && correctionsRecentlyReceived()) {
    message.correction_age_s = static_cast<float>(std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_valid_rtcm_.value()).count());
  }
  status_publisher_->publish(message);
  epoch.status_published = true;
}

void ZedF9rNode::publishGnssFixIfComplete(uint32_t i_tow_ms)
{
  const auto epoch_iterator = pending_epochs_.find(i_tow_ms);
  if (epoch_iterator == pending_epochs_.end() ||
    !epoch_iterator->second.pvt.has_value() ||
    !epoch_iterator->second.high_precision_position.has_value() || !fix_publisher_)
  {
    return;
  }

  const PendingEpoch & epoch = epoch_iterator->second;
  const NavPvt & pvt = epoch.pvt.value();
  const NavHpPosLlh & position = epoch.high_precision_position.value();
  const bool valid_fix = pvt.validGnssPosition() && !position.invalid_llh;

  sensor_msgs::msg::NavSatFix message;
  message.header.stamp = epoch.reception_stamp;
  message.header.frame_id = frame_id_;
  message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
    sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

  if (valid_fix) {
    message.latitude = position.latitude_deg;
    message.longitude = position.longitude_deg;
    message.altitude = position.ellipsoid_height_m;
    message.status.status = pvt.differentialSolution() ?
      sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_FIX;

    bool used_full_covariance = false;
    const auto covariance_iterator = covariances_.find(i_tow_ms);
    if (covariance_iterator != covariances_.end() && covariance_iterator->second.position_valid) {
      const std::array<double, 9> covariance = navCovarianceToEnu(covariance_iterator->second);
      const bool finite = std::all_of(
        covariance.begin(), covariance.end(), [](double value) {return std::isfinite(value);});
      if (finite && covariance[0U] >= 0.0 && covariance[4U] >= 0.0 && covariance[8U] >= 0.0) {
        message.position_covariance = covariance;
        message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
        used_full_covariance = true;
      }
    }
    if (!used_full_covariance) {
      const double horizontal_variance =
        position.horizontal_accuracy_m * position.horizontal_accuracy_m;
      const double vertical_variance = position.vertical_accuracy_m * position.vertical_accuracy_m;
      message.position_covariance = {{
        horizontal_variance, 0.0, 0.0,
        0.0, horizontal_variance, 0.0,
        0.0, 0.0, vertical_variance}};
      message.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
  } else {
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();
    message.latitude = not_a_number;
    message.longitude = not_a_number;
    message.altitude = not_a_number;
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  fix_publisher_->publish(message);
  pending_epochs_.erase(epoch_iterator);
  covariances_.erase(i_tow_ms);
}

void ZedF9rNode::publishImuSample(const ImuSample & sample)
{
  if (!imu_publisher_) {
    return;
  }
  const std::array<double, 3> angular_velocity = mapAxes(sample.angular_velocity);
  const std::array<double, 3> linear_acceleration = mapAxes(sample.linear_acceleration);

  sensor_msgs::msg::Imu message;
  message.header.stamp = now();
  message.header.frame_id = frame_id_;
  message.orientation_covariance[0U] = -1.0;
  message.angular_velocity.x = angular_velocity[0U];
  message.angular_velocity.y = angular_velocity[1U];
  message.angular_velocity.z = angular_velocity[2U];
  message.linear_acceleration.x = linear_acceleration[0U];
  message.linear_acceleration.y = linear_acceleration[1U];
  message.linear_acceleration.z = linear_acceleration[2U];
  imu_publisher_->publish(message);
}

void ZedF9rNode::onRtcm(const std_msgs::msg::UInt8MultiArray::SharedPtr message)
{
  if (message->data.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Mensagem RTCM vazia ignorada");
    return;
  }
  std::string error;
  if (!transport_ || !transport_->writeBytes(message->data, &error)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Falha ao enviar RTCM: %s", error.c_str());
    return;
  }
  ++rtcm_messages_sent_;
  rtcm_bytes_sent_ += message->data.size();
}

void ZedF9rNode::sendStartupRequests()
{
  std::string error;
  if (!transport_->writeBytes(buildMonVerPoll(), &error)) {
    RCLCPP_ERROR(get_logger(), "Falha ao enviar poll UBX-MON-VER: %s", error.c_str());
  }
  if (!apply_receiver_configuration_) {
    return;
  }

  std::vector<ConfigurationValue> values;
  values.push_back(booleanConfiguration(kCfgSfcoreUseSf, !disable_internal_sensor_fusion_));
  values.push_back(
    u1Configuration(
      kCfgRateNavPrio, static_cast<uint8_t>(priority_navigation_rate_hz_)));
  values.push_back(
    u2Configuration(kCfgRateMeas, static_cast<uint16_t>(measurement_rate_ms_)));
  if (enable_rtcm_diagnostics_) {
    values.push_back(u1Configuration(kCfgMsgoutUbxRxmRtcmUsb, 1U));
  }
  const std::vector<uint8_t> configuration = buildCfgValsetRam(values);
  if (!transport_->writeBytes(configuration, &error)) {
    RCLCPP_ERROR(get_logger(), "Falha ao enviar UBX-CFG-VALSET: %s", error.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(configuration_mutex_);
    configuration_ack_pending_ = true;
    configuration_ack_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(configuration_ack_timeout_ms_);
  }
}

void ZedF9rNode::checkConfigurationAcknowledgement()
{
  bool timed_out = false;
  {
    std::lock_guard<std::mutex> lock(configuration_mutex_);
    if (configuration_ack_pending_ &&
      std::chrono::steady_clock::now() >= configuration_ack_deadline_)
    {
      configuration_ack_pending_ = false;
      timed_out = true;
    }
  }
  if (timed_out) {
    RCLCPP_ERROR(
      get_logger(),
      "Timeout aguardando ACK/NAK do UBX-CFG-VALSET; recepcao de dados continua");
  }
}

void ZedF9rNode::logStatistics()
{
  const UbxParserStatistics statistics = parser_.statistics();
  RCLCPP_INFO(
    get_logger(),
    "UBX: validos=%" PRIu64 " invalidos=%" PRIu64 " desconhecidos=%" PRIu64
    "; RTCM TX=%" PRIu64 " msgs/%" PRIu64 " bytes; RTCM aceitos pelo receptor=%" PRIu64,
    statistics.valid_frames, statistics.invalid_frames, statistics.unknown_frames,
    rtcm_messages_sent_.load(), rtcm_bytes_sent_.load(), rtcm_receiver_messages_.load());
}

bool ZedF9rNode::correctionsRecentlyReceived() const
{
  if (!last_valid_rtcm_.has_value()) {
    return false;
  }
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_valid_rtcm_.value()).count() <=
         correction_received_timeout_s_;
}

void ZedF9rNode::prunePendingData()
{
  while (pending_epochs_.size() > 64U) {
    pending_epochs_.erase(pending_epochs_.begin());
  }
  while (covariances_.size() > 64U) {
    covariances_.erase(covariances_.begin());
  }
  while (imu_samples_.size() > 128U) {
    imu_samples_.erase(imu_samples_.begin());
  }
}

std::array<double, 3> ZedF9rNode::mapAxes(const std::array<double, 3> & native) const
{
  std::array<double, 3> mapped{};
  for (std::size_t output_axis = 0; output_axis < mapped.size(); ++output_axis) {
    const int selection = imu_axis_mapping_[output_axis];
    const std::size_t input_axis = static_cast<std::size_t>(std::abs(selection) - 1);
    mapped[output_axis] = selection > 0 ? native[input_axis] : -native[input_axis];
  }
  return mapped;
}

}  // namespace vida_zed_f9r_driver
