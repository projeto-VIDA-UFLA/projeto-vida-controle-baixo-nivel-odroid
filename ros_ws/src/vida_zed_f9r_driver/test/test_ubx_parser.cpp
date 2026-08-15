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

#include <array>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <vector>

#include "vida_zed_f9r_driver/ubx_parser.hpp"
#include "vida_zed_f9r_driver/ubx_protocol.hpp"

namespace vida_zed_f9r_driver
{

TEST(UbxParser, ParsesValidPacket)
{
  UbxParser parser;
  const std::vector<uint8_t> encoded = encodeUbx(kClassNav, kIdNavPvt, {1U, 2U, 3U});
  const std::vector<UbxFrame> frames = parser.feed(encoded);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0U].message_class, kClassNav);
  EXPECT_EQ(frames[0U].message_id, kIdNavPvt);
  EXPECT_EQ(frames[0U].payload, (std::vector<uint8_t>{1U, 2U, 3U}));
  EXPECT_EQ(parser.statistics().valid_frames, 1U);
}

TEST(UbxParser, RejectsInvalidChecksum)
{
  UbxParser parser;
  std::vector<uint8_t> encoded = encodeUbx(kClassNav, kIdNavPvt, {1U});
  encoded.back() ^= 0xffU;

  EXPECT_TRUE(parser.feed(encoded).empty());
  EXPECT_EQ(parser.statistics().invalid_frames, 1U);
}

TEST(UbxParser, SkipsNoiseBeforeSynchronization)
{
  UbxParser parser;
  std::vector<uint8_t> bytes{0U, 0xffU, kUbxSync1, 0U, 3U};
  const std::vector<uint8_t> encoded = encodeUbx(kClassMon, kIdMonVer);
  bytes.insert(bytes.end(), encoded.begin(), encoded.end());

  const std::vector<UbxFrame> frames = parser.feed(bytes);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0U].message_id, kIdMonVer);
}

TEST(UbxParser, AcceptsPacketSplitAcrossReads)
{
  UbxParser parser;
  const std::vector<uint8_t> encoded = encodeUbx(kClassNav, kIdNavPvt, {4U, 5U});

  EXPECT_TRUE(parser.feed(encoded.data(), 3U).empty());
  EXPECT_TRUE(parser.feed(encoded.data() + 3U, 2U).empty());
  const std::vector<UbxFrame> frames = parser.feed(
    encoded.data() + 5U, encoded.size() - 5U);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0U].payload, (std::vector<uint8_t>{4U, 5U}));
}

TEST(UbxParser, AcceptsMultiplePacketsInOneRead)
{
  UbxParser parser;
  std::vector<uint8_t> bytes = encodeUbx(kClassMon, kIdMonVer);
  const std::vector<uint8_t> second = encodeUbx(kClassNav, kIdNavPvt, {7U});
  bytes.insert(bytes.end(), second.begin(), second.end());

  const std::vector<UbxFrame> frames = parser.feed(bytes);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0U].message_class, kClassMon);
  EXPECT_EQ(frames[1U].message_class, kClassNav);
}

TEST(UbxParser, RejectsExcessivePayloadAndRecovers)
{
  UbxParser parser(32U);
  std::vector<uint8_t> bytes{kUbxSync1, kUbxSync2, kClassNav, kIdNavPvt, 33U, 0U};
  const std::vector<uint8_t> valid = encodeUbx(kClassMon, kIdMonVer);
  bytes.insert(bytes.end(), valid.begin(), valid.end());

  const std::vector<UbxFrame> frames = parser.feed(bytes);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0U].message_id, kIdMonVer);
  EXPECT_EQ(parser.statistics().invalid_frames, 1U);
}

TEST(UbxParser, RecoversAfterCorruptedFrame)
{
  UbxParser parser;
  std::vector<uint8_t> corrupt = encodeUbx(kClassNav, kIdNavPvt, {9U, 8U});
  corrupt[corrupt.size() - 2U] ^= 0x01U;
  const std::vector<uint8_t> valid = encodeUbx(kClassMon, kIdMonVer);
  corrupt.insert(corrupt.end(), valid.begin(), valid.end());

  const std::vector<UbxFrame> frames = parser.feed(corrupt);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0U].message_id, kIdMonVer);
  EXPECT_EQ(parser.statistics().invalid_frames, 1U);
}

TEST(UbxParser, BuildsMonVerPollWithValidChecksum)
{
  const std::vector<uint8_t> poll = buildMonVerPoll();
  const std::vector<uint8_t> expected{
    0xb5U, 0x62U, 0x0aU, 0x04U, 0x00U, 0x00U, 0x0eU, 0x34U};
  EXPECT_EQ(poll, expected);

  UbxParser parser;
  const std::vector<UbxFrame> frames = parser.feed(poll);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_TRUE(frames[0U].payload.empty());
}

TEST(UbxParser, SafelyCountsUnknownMessages)
{
  UbxParser parser;
  const std::vector<uint8_t> unknown = encodeUbx(0xf0U, 0x99U, {0xb5U, 0x62U});
  const std::vector<UbxFrame> frames = parser.feed(unknown);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(parser.statistics().valid_frames, 1U);
  EXPECT_EQ(parser.statistics().unknown_frames, 1U);
  EXPECT_EQ(parser.statistics().invalid_frames, 0U);
}

TEST(UbxParser, ParsesOptionalRealCapture)
{
  std::ifstream input("/tmp/f9r_capture.ubx", std::ios::binary);
  if (!input.good()) {
    GTEST_SKIP() << "/tmp/f9r_capture.ubx is not mounted in this container";
  }

  UbxParser parser;
  std::array<char, 137> input_block{};
  std::array<uint8_t, 137> parser_block{};
  uint64_t emitted = 0U;
  while (input.good()) {
    input.read(input_block.data(), static_cast<std::streamsize>(input_block.size()));
    const std::size_t count = static_cast<std::size_t>(input.gcount());
    std::transform(
      input_block.begin(), input_block.begin() + static_cast<std::ptrdiff_t>(count),
      parser_block.begin(), [](char value) {return static_cast<uint8_t>(value);});
    emitted += parser.feed(parser_block.data(), count).size();
  }
  EXPECT_EQ(emitted, 2829U);
  EXPECT_EQ(parser.statistics().valid_frames, 2829U);
  EXPECT_EQ(parser.statistics().invalid_frames, 0U);
}

}  // namespace vida_zed_f9r_driver
