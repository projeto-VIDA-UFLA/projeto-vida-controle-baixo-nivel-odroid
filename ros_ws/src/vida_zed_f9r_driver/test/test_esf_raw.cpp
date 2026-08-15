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

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "vida_zed_f9r_driver/ubx_protocol.hpp"

namespace vida_zed_f9r_driver
{
namespace
{

void appendU4(std::vector<uint8_t> & payload, uint32_t value)
{
  payload.push_back(static_cast<uint8_t>(value & 0xffU));
  payload.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
  payload.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
  payload.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
}

void appendMeasurement(
  std::vector<uint8_t> & payload, uint8_t type, int32_t value, uint32_t time_tag)
{
  const uint32_t encoded = (static_cast<uint32_t>(type) << 24U) |
    (static_cast<uint32_t>(value) & 0x00ffffffU);
  appendU4(payload, encoded);
  appendU4(payload, time_tag);
}

}  // namespace

TEST(UbxProtocol, ConvertsSigned24BitValues)
{
  EXPECT_EQ(signed24(0x000000U), 0);
  EXPECT_EQ(signed24(0x7fffffU), 8388607);
  EXPECT_EQ(signed24(0x800000U), -8388608);
  EXPECT_EQ(signed24(0xffffffU), -1);
  EXPECT_EQ(signed24(0x12ffffffU), -1);
}

TEST(UbxProtocol, ConvertsGyroscopeAndAccelerometerScales)
{
  constexpr double kPi = 3.14159265358979323846;
  EXPECT_NEAR(gyroscopeRadiansPerSecond(4096), kPi / 180.0, 1.0e-15);
  EXPECT_NEAR(gyroscopeRadiansPerSecond(-4096), -kPi / 180.0, 1.0e-15);
  EXPECT_DOUBLE_EQ(accelerationMetersPerSecondSquared(1024), 1.0);
  EXPECT_DOUBLE_EQ(accelerationMetersPerSecondSquared(-1024), -1.0);
}

TEST(UbxProtocol, ParsesCompleteEsfRawSampleWithSharedTimeTag)
{
  std::vector<uint8_t> payload(4U, 0U);
  constexpr uint32_t kTimeTag = 998877U;
  appendMeasurement(payload, kEsfGyroX, 4096, kTimeTag);
  appendMeasurement(payload, kEsfGyroY, -4096, kTimeTag);
  appendMeasurement(payload, kEsfGyroZ, 2048, kTimeTag);
  appendMeasurement(payload, kEsfAccelX, 1024, kTimeTag);
  appendMeasurement(payload, kEsfAccelY, -1024, kTimeTag);
  appendMeasurement(payload, kEsfAccelZ, 512, kTimeTag);

  const std::optional<std::vector<EsfRawMeasurement>> parsed = parseEsfRaw(payload);
  ASSERT_TRUE(parsed.has_value());
  ASSERT_EQ(parsed->size(), 6U);
  for (const EsfRawMeasurement & measurement : parsed.value()) {
    EXPECT_EQ(measurement.sensor_time_tag, kTimeTag);
  }
  EXPECT_EQ((*parsed)[0U].data_type, kEsfGyroX);
  EXPECT_EQ((*parsed)[1U].raw_value, -4096);
  EXPECT_EQ((*parsed)[4U].data_type, kEsfAccelY);
}

TEST(UbxProtocol, RejectsMalformedEsfRawPayload)
{
  EXPECT_FALSE(parseEsfRaw({0U, 0U, 0U}).has_value());
  EXPECT_FALSE(parseEsfRaw({0U, 0U, 0U, 0U, 1U}).has_value());
}

}  // namespace vida_zed_f9r_driver
