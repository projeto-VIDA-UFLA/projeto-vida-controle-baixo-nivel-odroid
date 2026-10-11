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

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "vida_ntrip_client/rtcm_parser.hpp"

namespace vida_ntrip_client
{
namespace
{

std::vector<uint8_t> makeFrame(uint16_t type, std::vector<uint8_t> trailing = {0x12, 0x34})
{
  std::vector<uint8_t> payload = {
    static_cast<uint8_t>(type >> 4U), static_cast<uint8_t>((type & 0x0fU) << 4U)};
  payload.insert(payload.end(), trailing.begin(), trailing.end());
  std::vector<uint8_t> frame = {
    0xd3, static_cast<uint8_t>((payload.size() >> 8U) & 0x03U),
    static_cast<uint8_t>(payload.size() & 0xffU)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint32_t crc = crc24q(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc >> 16U));
  frame.push_back(static_cast<uint8_t>(crc >> 8U));
  frame.push_back(static_cast<uint8_t>(crc));
  return frame;
}

}  // namespace

TEST(Crc24Q, MatchesPublishedCheckValue)
{
  const std::string input = "123456789";
  EXPECT_EQ(
    crc24q(reinterpret_cast<const uint8_t *>(input.data()), input.size()),
    0xcde703U);
}

TEST(RtcmParser, ParsesIncrementallyAcrossEveryBoundary)
{
  const std::vector<uint8_t> frame = makeFrame(1006U);
  for (std::size_t split = 1U; split < frame.size(); ++split) {
    RtcmParser parser;
    parser.feed(frame.data(), split);
    EXPECT_EQ(parser.statistics().valid_frames, 0U);
    parser.feed(frame.data() + split, frame.size() - split);
    EXPECT_EQ(parser.statistics().valid_frames, 1U);
    EXPECT_EQ(parser.statistics().message_type_counts.at(1006U), 1U);
  }
}

TEST(RtcmParser, CountsValidAndInvalidCrc)
{
  RtcmParser parser;
  const std::vector<uint8_t> valid = makeFrame(1074U);
  std::vector<uint8_t> invalid = makeFrame(1084U);
  invalid.back() ^= 0x01U;
  parser.feed(valid);
  parser.feed(invalid);
  EXPECT_EQ(parser.statistics().valid_frames, 1U);
  EXPECT_EQ(parser.statistics().invalid_crc_frames, 1U);
  EXPECT_EQ(parser.statistics().message_type_counts.at(1074U), 1U);
}

TEST(RtcmParser, RecoversAfterNoiseAndCorruption)
{
  RtcmParser parser;
  std::vector<uint8_t> stream = {0x00, 0xaa, 0x55, 0xd2};
  std::vector<uint8_t> corrupt = makeFrame(1006U, {0x01, 0x02});
  corrupt[corrupt.size() - 2U] ^= 0x80U;
  const std::vector<uint8_t> recovered = makeFrame(1230U, {0x03, 0x04});
  stream.insert(stream.end(), corrupt.begin(), corrupt.end());
  stream.insert(stream.end(), {0x11, 0x22, 0x33});
  stream.insert(stream.end(), recovered.begin(), recovered.end());
  parser.feed(stream);
  EXPECT_EQ(parser.statistics().invalid_crc_frames, 1U);
  EXPECT_EQ(parser.statistics().valid_frames, 1U);
  EXPECT_EQ(parser.statistics().message_type_counts.at(1230U), 1U);
}

TEST(RtcmParser, IdentifiesObservedMessageTypes)
{
  constexpr uint16_t kTypes[] = {1006, 1074, 1084, 1094, 1124, 1230};
  std::vector<uint8_t> stream;
  for (const uint16_t type : kTypes) {
    const std::vector<uint8_t> frame = makeFrame(type);
    stream.insert(stream.end(), frame.begin(), frame.end());
  }
  RtcmParser parser;
  parser.feed(stream);
  EXPECT_EQ(parser.statistics().valid_frames, 6U);
  for (const uint16_t type : kTypes) {
    EXPECT_EQ(parser.statistics().message_type_counts.at(type), 1U);
  }
}

TEST(RtcmParser, KeepsDefensiveBufferBound)
{
  RtcmParser parser(1029U);
  const std::vector<uint8_t> noise(10000U, 0x55U);
  parser.feed(noise);
  EXPECT_LT(parser.bufferedBytes(), 3U);
}

}  // namespace vida_ntrip_client
