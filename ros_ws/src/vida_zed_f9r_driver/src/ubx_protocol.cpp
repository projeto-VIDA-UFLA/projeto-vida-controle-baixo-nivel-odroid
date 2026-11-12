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

#include "vida_zed_f9r_driver/ubx_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace vida_zed_f9r_driver
{
namespace
{

uint16_t readU2(const std::vector<uint8_t> & data, std::size_t offset)
{
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1U]) << 8U);
}

uint32_t readU4(const std::vector<uint8_t> & data, std::size_t offset)
{
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1U]) << 8U) |
         (static_cast<uint32_t>(data[offset + 2U]) << 16U) |
         (static_cast<uint32_t>(data[offset + 3U]) << 24U);
}

int32_t readI4(const std::vector<uint8_t> & data, std::size_t offset)
{
  const uint32_t unsigned_value = readU4(data, offset);
  int32_t signed_value = 0;
  std::memcpy(&signed_value, &unsigned_value, sizeof(signed_value));
  return signed_value;
}

int16_t readI2(const std::vector<uint8_t> & data, std::size_t offset)
{
  const uint16_t unsigned_value = readU2(data, offset);
  int16_t signed_value = 0;
  std::memcpy(&signed_value, &unsigned_value, sizeof(signed_value));
  return signed_value;
}

float readR4(const std::vector<uint8_t> & data, std::size_t offset)
{
  const uint32_t bits = readU4(data, offset);
  float value = 0.0F;
  static_assert(sizeof(value) == sizeof(bits), "UBX R4 requires a 32-bit float");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

int8_t readI1(const std::vector<uint8_t> & data, std::size_t offset)
{
  int8_t value = 0;
  std::memcpy(&value, &data[offset], sizeof(value));
  return value;
}

void appendU4(std::vector<uint8_t> & destination, uint32_t value)
{
  destination.push_back(static_cast<uint8_t>(value & 0xffU));
  destination.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
  destination.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
  destination.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
}

std::string fixedString(
  const std::vector<uint8_t> & payload, std::size_t offset, std::size_t width)
{
  const auto begin = payload.begin() + static_cast<std::ptrdiff_t>(offset);
  const auto end = begin + static_cast<std::ptrdiff_t>(width);
  const auto nul = std::find(begin, end, uint8_t{0});
  return std::string(begin, nul);
}

}  // namespace

bool UbxChecksum::operator==(const UbxChecksum & other) const
{
  return a == other.a && b == other.b;
}

UbxChecksum calculateChecksum(const uint8_t * data, std::size_t size)
{
  UbxChecksum checksum;
  for (std::size_t index = 0; index < size; ++index) {
    checksum.a = static_cast<uint8_t>(checksum.a + data[index]);
    checksum.b = static_cast<uint8_t>(checksum.b + checksum.a);
  }
  return checksum;
}

std::vector<uint8_t> encodeUbx(
  uint8_t message_class, uint8_t message_id, const std::vector<uint8_t> & payload)
{
  if (payload.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::length_error("UBX payload exceeds the 16-bit length field");
  }

  const auto payload_size = static_cast<uint16_t>(payload.size());
  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 8U);
  frame.push_back(kUbxSync1);
  frame.push_back(kUbxSync2);
  frame.push_back(message_class);
  frame.push_back(message_id);
  frame.push_back(static_cast<uint8_t>(payload_size & 0xffU));
  frame.push_back(static_cast<uint8_t>((payload_size >> 8U) & 0xffU));
  frame.insert(frame.end(), payload.begin(), payload.end());

  const UbxChecksum checksum = calculateChecksum(frame.data() + 2U, frame.size() - 2U);
  frame.push_back(checksum.a);
  frame.push_back(checksum.b);
  return frame;
}

bool isKnownMessage(uint8_t message_class, uint8_t message_id)
{
  if (message_class == kClassAck) {
    return message_id == kIdAckAck || message_id == kIdAckNak;
  }
  if (message_class == kClassCfg) {
    return message_id == kIdCfgValset;
  }
  if (message_class == kClassMon) {
    return message_id == kIdMonVer;
  }
  if (message_class == kClassNav) {
    return message_id == 0x05U || message_id == kIdNavPvt ||
           message_id == kIdNavHpPosLlh || message_id == 0x35U ||
           message_id == kIdNavCov || message_id == 0x43U;
  }
  if (message_class == kClassRxm) {
    return message_id == 0x15U || message_id == kIdRxmRtcm;
  }
  if (message_class == kClassEsf) {
    return message_id == 0x02U || message_id == kIdEsfRaw ||
           message_id == 0x10U || message_id == 0x14U || message_id == 0x15U;
  }
  return false;
}

ConfigurationValue booleanConfiguration(uint32_t key, bool value)
{
  return ConfigurationValue{key, {static_cast<uint8_t>(value ? 1U : 0U)}};
}

ConfigurationValue u1Configuration(uint32_t key, uint8_t value)
{
  return ConfigurationValue{key, {value}};
}

ConfigurationValue u2Configuration(uint32_t key, uint16_t value)
{
  // UBX e little-endian.
  return ConfigurationValue{
    key, {static_cast<uint8_t>(value & 0xFFU), static_cast<uint8_t>((value >> 8U) & 0xFFU)}};
}

std::vector<uint8_t> buildCfgValsetRam(const std::vector<ConfigurationValue> & values)
{
  if (values.size() > 64U) {
    throw std::length_error("UBX-CFG-VALSET accepts at most 64 key-value pairs");
  }

  std::vector<uint8_t> payload{0x00U, 0x01U, 0x00U, 0x00U};
  for (const ConfigurationValue & item : values) {
    if (item.value.empty()) {
      throw std::invalid_argument("UBX configuration value cannot be empty");
    }
    appendU4(payload, item.key);
    payload.insert(payload.end(), item.value.begin(), item.value.end());
  }
  return encodeUbx(kClassCfg, kIdCfgValset, payload);
}

std::vector<uint8_t> buildMonVerPoll()
{
  return encodeUbx(kClassMon, kIdMonVer);
}

bool NavPvt::gnssFixOk() const
{
  return (flags & 0x01U) != 0U;
}

bool NavPvt::differentialSolution() const
{
  return (flags & 0x02U) != 0U;
}

uint8_t NavPvt::carrierSolution() const
{
  return static_cast<uint8_t>((flags >> 6U) & 0x03U);
}

uint8_t NavPvt::correctionAgeCode() const
{
  return static_cast<uint8_t>((flags3 >> 1U) & 0x0fU);
}

bool NavPvt::validGnssPosition() const
{
  const bool invalid_llh = (flags3 & 0x01U) != 0U;
  return gnssFixOk() && !invalid_llh && (fix_type == 2U || fix_type == 3U);
}

std::optional<NavPvt> parseNavPvt(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 92U) {
    return std::nullopt;
  }
  NavPvt result;
  result.i_tow_ms = readU4(payload, 0U);
  result.year = readU2(payload, 4U);
  result.month = payload[6U];
  result.day = payload[7U];
  result.hour = payload[8U];
  result.minute = payload[9U];
  result.second = payload[10U];
  result.valid = payload[11U];
  result.time_accuracy_ns = readU4(payload, 12U);
  result.nano_ns = readI4(payload, 16U);
  result.fix_type = payload[20U];
  result.flags = payload[21U];
  result.flags2 = payload[22U];
  result.num_sv = payload[23U];
  result.longitude_1e7_deg = readI4(payload, 24U);
  result.latitude_1e7_deg = readI4(payload, 28U);
  result.ellipsoid_height_mm = readI4(payload, 32U);
  result.msl_height_mm = readI4(payload, 36U);
  result.horizontal_accuracy_mm = readU4(payload, 40U);
  result.vertical_accuracy_mm = readU4(payload, 44U);
  result.velocity_north_mm_s = readI4(payload, 48U);
  result.velocity_east_mm_s = readI4(payload, 52U);
  result.velocity_down_mm_s = readI4(payload, 56U);
  result.ground_speed_mm_s = readI4(payload, 60U);
  result.heading_of_motion_1e5_deg = readI4(payload, 64U);
  result.speed_accuracy_mm_s = readU4(payload, 68U);
  result.heading_accuracy_1e5_deg = readU4(payload, 72U);
  result.position_dop_1e2 = readU2(payload, 76U);
  result.flags3 = readU2(payload, 78U);
  std::copy_n(payload.begin() + 80, result.reserved0.size(), result.reserved0.begin());
  result.heading_of_vehicle_1e5_deg = readI4(payload, 84U);
  result.magnetic_declination_1e2_deg = readI2(payload, 88U);
  result.magnetic_accuracy_1e2_deg = readU2(payload, 90U);
  return result;
}

std::optional<NavHpPosLlh> parseNavHpPosLlh(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 36U || payload[0U] != 0U) {
    return std::nullopt;
  }
  NavHpPosLlh result;
  result.invalid_llh = (payload[3U] & 0x01U) != 0U;
  result.i_tow_ms = readU4(payload, 4U);
  result.longitude_deg = static_cast<double>(readI4(payload, 8U)) * 1.0e-7 +
    static_cast<double>(readI1(payload, 24U)) * 1.0e-9;
  result.latitude_deg = static_cast<double>(readI4(payload, 12U)) * 1.0e-7 +
    static_cast<double>(readI1(payload, 25U)) * 1.0e-9;
  result.ellipsoid_height_m = static_cast<double>(readI4(payload, 16U)) * 1.0e-3 +
    static_cast<double>(readI1(payload, 26U)) * 1.0e-4;
  result.horizontal_accuracy_m = static_cast<double>(readU4(payload, 28U)) * 1.0e-4;
  result.vertical_accuracy_m = static_cast<double>(readU4(payload, 32U)) * 1.0e-4;
  return result;
}

std::optional<NavCov> parseNavCov(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 64U || payload[4U] != 0U) {
    return std::nullopt;
  }
  NavCov result;
  result.i_tow_ms = readU4(payload, 0U);
  result.position_valid = payload[5U] != 0U;
  result.nn = readR4(payload, 16U);
  result.ne = readR4(payload, 20U);
  result.nd = readR4(payload, 24U);
  result.ee = readR4(payload, 28U);
  result.ed = readR4(payload, 32U);
  result.dd = readR4(payload, 36U);
  return result;
}

std::optional<std::vector<EsfRawMeasurement>> parseEsfRaw(
  const std::vector<uint8_t> & payload)
{
  if (payload.size() < 4U || (payload.size() - 4U) % 8U != 0U) {
    return std::nullopt;
  }
  std::vector<EsfRawMeasurement> measurements;
  measurements.reserve((payload.size() - 4U) / 8U);
  for (std::size_t offset = 4U; offset < payload.size(); offset += 8U) {
    const uint32_t data = readU4(payload, offset);
    measurements.push_back(
      EsfRawMeasurement{
        static_cast<uint8_t>(data >> 24U), signed24(data), readU4(payload, offset + 4U)});
  }
  return measurements;
}

std::optional<MonVersion> parseMonVersion(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 40U || (payload.size() - 40U) % 30U != 0U) {
    return std::nullopt;
  }
  MonVersion result;
  result.software = fixedString(payload, 0U, 30U);
  result.hardware = fixedString(payload, 30U, 10U);
  for (std::size_t offset = 40U; offset < payload.size(); offset += 30U) {
    std::string extension = fixedString(payload, offset, 30U);
    if (!extension.empty()) {
      result.extensions.push_back(std::move(extension));
    }
  }
  return result;
}

std::string MonVersion::extensionValue(const std::string & prefix) const
{
  for (const std::string & extension : extensions) {
    if (extension.rfind(prefix, 0U) == 0U) {
      return extension.substr(prefix.size());
    }
  }
  return {};
}

std::optional<RxmRtcm> parseRxmRtcm(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 8U || payload[0U] != 0x02U) {
    return std::nullopt;
  }
  RxmRtcm result;
  result.crc_failed = (payload[1U] & 0x01U) != 0U;
  result.message_used = static_cast<uint8_t>((payload[1U] >> 1U) & 0x03U);
  result.reference_station = readU2(payload, 4U);
  result.message_type = readU2(payload, 6U);
  return result;
}

int32_t signed24(uint32_t value)
{
  value &= 0x00ffffffU;
  if ((value & 0x00800000U) != 0U) {
    value |= 0xff000000U;
  }
  int32_t signed_value = 0;
  std::memcpy(&signed_value, &value, sizeof(signed_value));
  return signed_value;
}

double gyroscopeRadiansPerSecond(int32_t raw_value)
{
  constexpr double kPi = 3.14159265358979323846;
  return std::ldexp(static_cast<double>(raw_value), -12) * kPi / 180.0;
}

double accelerationMetersPerSecondSquared(int32_t raw_value)
{
  return std::ldexp(static_cast<double>(raw_value), -10);
}

float correctionAgeLowerBoundSeconds(uint8_t age_code)
{
  constexpr std::array<float, 16> kLowerBounds{{
    std::numeric_limits<float>::quiet_NaN(), 0.0F, 1.0F, 2.0F,
    5.0F, 10.0F, 15.0F, 20.0F,
    30.0F, 45.0F, 60.0F, 90.0F,
    120.0F, 120.0F, 120.0F, 120.0F}};
  return kLowerBounds[age_code & 0x0fU];
}

std::array<double, 9> navCovarianceToEnu(const NavCov & covariance)
{
  return {{
    covariance.ee, covariance.ne, -covariance.ed,
    covariance.ne, covariance.nn, -covariance.nd,
    -covariance.ed, -covariance.nd, covariance.dd}};
}

}  // namespace vida_zed_f9r_driver
