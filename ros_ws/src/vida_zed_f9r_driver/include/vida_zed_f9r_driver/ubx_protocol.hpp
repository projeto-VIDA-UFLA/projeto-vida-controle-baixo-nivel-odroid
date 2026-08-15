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

#ifndef VIDA_ZED_F9R_DRIVER__UBX_PROTOCOL_HPP_
#define VIDA_ZED_F9R_DRIVER__UBX_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace vida_zed_f9r_driver
{

constexpr uint8_t kUbxSync1 = 0xb5;
constexpr uint8_t kUbxSync2 = 0x62;

constexpr uint8_t kClassAck = 0x05;
constexpr uint8_t kIdAckNak = 0x00;
constexpr uint8_t kIdAckAck = 0x01;
constexpr uint8_t kClassCfg = 0x06;
constexpr uint8_t kIdCfgValset = 0x8a;
constexpr uint8_t kClassMon = 0x0a;
constexpr uint8_t kIdMonVer = 0x04;
constexpr uint8_t kClassNav = 0x01;
constexpr uint8_t kIdNavPvt = 0x07;
constexpr uint8_t kIdNavHpPosLlh = 0x14;
constexpr uint8_t kIdNavCov = 0x36;
constexpr uint8_t kClassEsf = 0x10;
constexpr uint8_t kIdEsfRaw = 0x03;
constexpr uint8_t kClassRxm = 0x02;
constexpr uint8_t kIdRxmRtcm = 0x32;

constexpr uint32_t kCfgSfcoreUseSf = 0x10080001U;
constexpr uint32_t kCfgRateNavPrio = 0x20210004U;
constexpr uint32_t kCfgMsgoutUbxRxmRtcmUsb = 0x2091026bU;

constexpr uint8_t kEsfGyroZ = 5;
constexpr uint8_t kEsfGyroY = 13;
constexpr uint8_t kEsfGyroX = 14;
constexpr uint8_t kEsfAccelX = 16;
constexpr uint8_t kEsfAccelY = 17;
constexpr uint8_t kEsfAccelZ = 18;

struct UbxFrame
{
  uint8_t message_class{0};
  uint8_t message_id{0};
  std::vector<uint8_t> payload;
};

struct UbxChecksum
{
  uint8_t a{0};
  uint8_t b{0};

  bool operator==(const UbxChecksum & other) const;
};

UbxChecksum calculateChecksum(const uint8_t * data, std::size_t size);
std::vector<uint8_t> encodeUbx(
  uint8_t message_class, uint8_t message_id,
  const std::vector<uint8_t> & payload = {});
bool isKnownMessage(uint8_t message_class, uint8_t message_id);

struct ConfigurationValue
{
  uint32_t key{0};
  std::vector<uint8_t> value;
};

ConfigurationValue booleanConfiguration(uint32_t key, bool value);
ConfigurationValue u1Configuration(uint32_t key, uint8_t value);
std::vector<uint8_t> buildCfgValsetRam(const std::vector<ConfigurationValue> & values);
std::vector<uint8_t> buildMonVerPoll();

struct NavPvt
{
  uint32_t i_tow_ms{0};
  uint8_t fix_type{0};
  uint8_t flags{0};
  uint8_t num_sv{0};
  uint32_t horizontal_accuracy_mm{0};
  uint32_t vertical_accuracy_mm{0};
  uint16_t flags3{0};

  bool gnssFixOk() const;
  bool differentialSolution() const;
  uint8_t carrierSolution() const;
  uint8_t correctionAgeCode() const;
  bool validGnssPosition() const;
};

struct NavHpPosLlh
{
  uint32_t i_tow_ms{0};
  bool invalid_llh{true};
  double longitude_deg{0.0};
  double latitude_deg{0.0};
  double ellipsoid_height_m{0.0};
  double horizontal_accuracy_m{0.0};
  double vertical_accuracy_m{0.0};
};

struct NavCov
{
  uint32_t i_tow_ms{0};
  bool position_valid{false};
  float nn{0.0F};
  float ne{0.0F};
  float nd{0.0F};
  float ee{0.0F};
  float ed{0.0F};
  float dd{0.0F};
};

struct EsfRawMeasurement
{
  uint8_t data_type{0};
  int32_t raw_value{0};
  uint32_t sensor_time_tag{0};
};

struct MonVersion
{
  std::string software;
  std::string hardware;
  std::vector<std::string> extensions;

  std::string extensionValue(const std::string & prefix) const;
};

struct RxmRtcm
{
  bool crc_failed{false};
  uint8_t message_used{0};
  uint16_t reference_station{0};
  uint16_t message_type{0};
};

std::optional<NavPvt> parseNavPvt(const std::vector<uint8_t> & payload);
std::optional<NavHpPosLlh> parseNavHpPosLlh(const std::vector<uint8_t> & payload);
std::optional<NavCov> parseNavCov(const std::vector<uint8_t> & payload);
std::optional<std::vector<EsfRawMeasurement>> parseEsfRaw(
  const std::vector<uint8_t> & payload);
std::optional<MonVersion> parseMonVersion(const std::vector<uint8_t> & payload);
std::optional<RxmRtcm> parseRxmRtcm(const std::vector<uint8_t> & payload);

int32_t signed24(uint32_t value);
double gyroscopeRadiansPerSecond(int32_t raw_value);
double accelerationMetersPerSecondSquared(int32_t raw_value);
float correctionAgeLowerBoundSeconds(uint8_t age_code);
std::array<double, 9> navCovarianceToEnu(const NavCov & covariance);

}  // namespace vida_zed_f9r_driver

#endif  // VIDA_ZED_F9R_DRIVER__UBX_PROTOCOL_HPP_
