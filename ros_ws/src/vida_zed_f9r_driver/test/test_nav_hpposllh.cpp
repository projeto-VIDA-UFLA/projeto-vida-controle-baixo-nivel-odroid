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
#include <cstring>
#include <vector>

#include "vida_zed_f9r_driver/ubx_parser.hpp"
#include "vida_zed_f9r_driver/ubx_protocol.hpp"

namespace vida_zed_f9r_driver
{
namespace
{

void putU4(std::vector<uint8_t> & payload, std::size_t offset, uint32_t value)
{
  payload[offset] = static_cast<uint8_t>(value & 0xffU);
  payload[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xffU);
  payload[offset + 2U] = static_cast<uint8_t>((value >> 16U) & 0xffU);
  payload[offset + 3U] = static_cast<uint8_t>((value >> 24U) & 0xffU);
}

void putI4(std::vector<uint8_t> & payload, std::size_t offset, int32_t value)
{
  uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  putU4(payload, offset, bits);
}

void putR4(std::vector<uint8_t> & payload, std::size_t offset, float value)
{
  uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  putU4(payload, offset, bits);
}

}  // namespace

TEST(UbxProtocol, ConvertsNavHpPosLlhWithSignedHighPrecisionFields)
{
  std::vector<uint8_t> payload(36U, 0U);
  putU4(payload, 4U, 123456U);
  putI4(payload, 8U, -739999999);
  putI4(payload, 12U, -225000001);
  putI4(payload, 16U, 12345);
  payload[24U] = static_cast<uint8_t>(-50);
  payload[25U] = static_cast<uint8_t>(25);
  payload[26U] = static_cast<uint8_t>(-5);
  putU4(payload, 28U, 123U);
  putU4(payload, 32U, 456U);

  const std::optional<NavHpPosLlh> result = parseNavHpPosLlh(payload);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->i_tow_ms, 123456U);
  EXPECT_NEAR(result->longitude_deg, -73.99999995, 1.0e-12);
  EXPECT_NEAR(result->latitude_deg, -22.500000075, 1.0e-12);
  EXPECT_NEAR(result->ellipsoid_height_m, 12.3445, 1.0e-12);
  EXPECT_NEAR(result->horizontal_accuracy_m, 0.0123, 1.0e-12);
  EXPECT_NEAR(result->vertical_accuracy_m, 0.0456, 1.0e-12);
}

TEST(UbxProtocol, ExtractsCarrierSolutionFromNavPvt)
{
  std::vector<uint8_t> payload(92U, 0U);
  payload[20U] = 3U;
  payload[21U] = static_cast<uint8_t>(0x01U | 0x02U | (2U << 6U));
  payload[23U] = 21U;
  putU4(payload, 40U, 15U);
  putU4(payload, 44U, 25U);
  payload[78U] = static_cast<uint8_t>(3U << 1U);

  const std::optional<NavPvt> result = parseNavPvt(payload);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->carrierSolution(), 2U);
  EXPECT_TRUE(result->gnssFixOk());
  EXPECT_TRUE(result->differentialSolution());
  EXPECT_TRUE(result->validGnssPosition());
  EXPECT_EQ(result->correctionAgeCode(), 3U);
  EXPECT_FLOAT_EQ(correctionAgeLowerBoundSeconds(result->correctionAgeCode()), 2.0F);
}

TEST(UbxProtocol, ParsesEveryNavPvtField)
{
  std::vector<uint8_t> payload(92U, 0U);
  putU4(payload, 0U, 345678U);
  payload[4U] = 0xeaU;  // 2026
  payload[5U] = 0x07U;
  payload[6U] = 11U;
  payload[7U] = 12U;
  payload[8U] = 13U;
  payload[9U] = 14U;
  payload[10U] = 15U;
  payload[11U] = 0x07U;
  putU4(payload, 12U, 31U);
  putI4(payload, 16U, -123456);
  payload[20U] = 3U;
  payload[21U] = 0x83U;
  payload[22U] = 0xe0U;
  payload[23U] = 27U;
  putI4(payload, 24U, -471234567);
  putI4(payload, 28U, -229876543);
  putI4(payload, 32U, 850123);
  putI4(payload, 36U, 856789);
  putU4(payload, 40U, 14U);
  putU4(payload, 44U, 21U);
  putI4(payload, 48U, -1500);
  putI4(payload, 52U, 2500);
  putI4(payload, 56U, -30);
  putI4(payload, 60U, 2915);
  putI4(payload, 64U, 12345678);
  putU4(payload, 68U, 45U);
  putU4(payload, 72U, 98765U);
  payload[76U] = 0x7bU;  // 123
  payload[78U] = 0x06U;
  payload[80U] = 1U;
  payload[83U] = 4U;
  putI4(payload, 84U, -4500000);
  payload[88U] = 0x38U;  // -2248
  payload[89U] = 0xf7U;
  payload[90U] = 0x2cU;  // 300
  payload[91U] = 0x01U;

  const std::optional<NavPvt> result = parseNavPvt(payload);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->i_tow_ms, 345678U);
  EXPECT_EQ(result->year, 2026U);
  EXPECT_EQ(result->month, 11U);
  EXPECT_EQ(result->day, 12U);
  EXPECT_EQ(result->hour, 13U);
  EXPECT_EQ(result->minute, 14U);
  EXPECT_EQ(result->second, 15U);
  EXPECT_EQ(result->valid, 0x07U);
  EXPECT_EQ(result->time_accuracy_ns, 31U);
  EXPECT_EQ(result->nano_ns, -123456);
  EXPECT_EQ(result->fix_type, 3U);
  EXPECT_EQ(result->flags, 0x83U);
  EXPECT_EQ(result->flags2, 0xe0U);
  EXPECT_EQ(result->num_sv, 27U);
  EXPECT_EQ(result->longitude_1e7_deg, -471234567);
  EXPECT_EQ(result->latitude_1e7_deg, -229876543);
  EXPECT_EQ(result->ellipsoid_height_mm, 850123);
  EXPECT_EQ(result->msl_height_mm, 856789);
  EXPECT_EQ(result->horizontal_accuracy_mm, 14U);
  EXPECT_EQ(result->vertical_accuracy_mm, 21U);
  EXPECT_EQ(result->velocity_north_mm_s, -1500);
  EXPECT_EQ(result->velocity_east_mm_s, 2500);
  EXPECT_EQ(result->velocity_down_mm_s, -30);
  EXPECT_EQ(result->ground_speed_mm_s, 2915);
  EXPECT_EQ(result->heading_of_motion_1e5_deg, 12345678);
  EXPECT_EQ(result->speed_accuracy_mm_s, 45U);
  EXPECT_EQ(result->heading_accuracy_1e5_deg, 98765U);
  EXPECT_EQ(result->position_dop_1e2, 123U);
  EXPECT_EQ(result->flags3, 0x06U);
  EXPECT_EQ(result->reserved0[0U], 1U);
  EXPECT_EQ(result->reserved0[3U], 4U);
  EXPECT_EQ(result->heading_of_vehicle_1e5_deg, -4500000);
  EXPECT_EQ(result->magnetic_declination_1e2_deg, -2248);
  EXPECT_EQ(result->magnetic_accuracy_1e2_deg, 300U);
}

TEST(UbxProtocol, RejectsDeadReckoningPositionAsPureGnss)
{
  NavPvt pvt;
  pvt.fix_type = 4U;
  pvt.flags = 0x01U;
  EXPECT_FALSE(pvt.validGnssPosition());
}

TEST(UbxProtocol, BuildsCfgValsetForRamOnly)
{
  const std::vector<uint8_t> packet = buildCfgValsetRam(
    {
      booleanConfiguration(kCfgSfcoreUseSf, false),
      u1Configuration(kCfgRateNavPrio, 0U),
      u1Configuration(kCfgMsgoutUbxRxmRtcmUsb, 1U)});

  UbxParser parser;
  const std::vector<UbxFrame> frames = parser.feed(packet);
  ASSERT_EQ(frames.size(), 1U);
  ASSERT_GE(frames[0U].payload.size(), 4U);
  EXPECT_EQ(frames[0U].message_class, kClassCfg);
  EXPECT_EQ(frames[0U].message_id, kIdCfgValset);
  EXPECT_EQ(frames[0U].payload[0U], 0U);
  EXPECT_EQ(frames[0U].payload[1U], 0x01U);
  EXPECT_EQ(frames[0U].payload[2U], 0U);
  EXPECT_EQ(frames[0U].payload[3U], 0U);
}

TEST(UbxProtocol, SerializesPriorityNavigationRateAsU1)
{
  const std::vector<uint8_t> packet = buildCfgValsetRam(
    {
      booleanConfiguration(kCfgSfcoreUseSf, false),
      u1Configuration(kCfgRateNavPrio, 0U)});

  UbxParser parser;
  const std::vector<UbxFrame> frames = parser.feed(packet);
  ASSERT_EQ(frames.size(), 1U);
  ASSERT_EQ(frames[0U].payload.size(), 14U);

  // VALSET v0, RAM only, then CFG-SFCORE-USE_SF=false.
  EXPECT_EQ(frames[0U].payload[0U], 0U);
  EXPECT_EQ(frames[0U].payload[1U], 0x01U);
  EXPECT_EQ(frames[0U].payload[2U], 0U);
  EXPECT_EQ(frames[0U].payload[3U], 0U);
  EXPECT_EQ(frames[0U].payload[4U], 0x01U);
  EXPECT_EQ(frames[0U].payload[5U], 0x00U);
  EXPECT_EQ(frames[0U].payload[6U], 0x08U);
  EXPECT_EQ(frames[0U].payload[7U], 0x10U);
  EXPECT_EQ(frames[0U].payload[8U], 0U);

  // CFG-RATE-NAV_PRIO key 0x20210004, little-endian, followed by one U1 byte.
  EXPECT_EQ(frames[0U].payload[9U], 0x04U);
  EXPECT_EQ(frames[0U].payload[10U], 0x00U);
  EXPECT_EQ(frames[0U].payload[11U], 0x21U);
  EXPECT_EQ(frames[0U].payload[12U], 0x20U);
  EXPECT_EQ(frames[0U].payload[13U], 0U);
}

TEST(UbxProtocol, ConvertsNavCovFromNedToEnu)
{
  std::vector<uint8_t> payload(64U, 0U);
  payload[5U] = 1U;
  putR4(payload, 16U, 1.0F);
  putR4(payload, 20U, 2.0F);
  putR4(payload, 24U, 3.0F);
  putR4(payload, 28U, 4.0F);
  putR4(payload, 32U, 5.0F);
  putR4(payload, 36U, 6.0F);

  const std::optional<NavCov> parsed = parseNavCov(payload);
  ASSERT_TRUE(parsed.has_value());
  const std::array<double, 9> enu = navCovarianceToEnu(parsed.value());
  const std::array<double, 9> expected{{4.0, 2.0, -5.0, 2.0, 1.0, -3.0, -5.0, -3.0, 6.0}};
  EXPECT_EQ(enu, expected);
}

TEST(UbxProtocol, SerializesMeasurementRateAsU2LittleEndian)
{
  const ConfigurationValue value = u2Configuration(kCfgRateMeas, 500U);
  EXPECT_EQ(value.key, 0x30210001U);
  ASSERT_EQ(value.value.size(), 2U);
  EXPECT_EQ(value.value[0U], 0xF4U);  // 500 = 0x01F4
  EXPECT_EQ(value.value[1U], 0x01U);

  const std::vector<uint8_t> packet = buildCfgValsetRam({value});
  UbxParser parser;
  const std::vector<UbxFrame> frames = parser.feed(packet);
  ASSERT_EQ(frames.size(), 1U);
  ASSERT_EQ(frames[0U].payload.size(), 4U + 4U + 2U);  // cabecalho + chave + U2
}

}  // namespace vida_zed_f9r_driver
