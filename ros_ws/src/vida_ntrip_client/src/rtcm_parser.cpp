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

#include "vida_ntrip_client/rtcm_parser.hpp"

#include <algorithm>
#include <stdexcept>

namespace vida_ntrip_client
{

uint32_t crc24q(const uint8_t * data, std::size_t size)
{
  uint32_t crc = 0U;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<uint32_t>(data[index]) << 16U;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1U;
      if ((crc & 0x1000000U) != 0U) {
        crc ^= 0x1864cfbU;
      }
    }
  }
  return crc & 0xffffffU;
}

RtcmParser::RtcmParser(std::size_t max_buffer_bytes)
: max_buffer_bytes_(max_buffer_bytes)
{
  if (max_buffer_bytes_ < 1029U) {
    throw std::invalid_argument("RTCM parser buffer must hold one maximum-size frame");
  }
  buffer_.reserve(max_buffer_bytes_);
}

void RtcmParser::feed(const std::vector<uint8_t> & data)
{
  feed(data.data(), data.size());
}

void RtcmParser::feed(const uint8_t * data, std::size_t size)
{
  if (data == nullptr && size != 0U) {
    return;
  }
  std::size_t offset = 0U;
  while (offset < size) {
    const std::size_t available = max_buffer_bytes_ - buffer_.size();
    if (available == 0U) {
      parseAvailable();
      if (buffer_.size() == max_buffer_bytes_) {
        buffer_.erase(buffer_.begin());
      }
      continue;
    }
    const std::size_t count = std::min(available, size - offset);
    buffer_.insert(buffer_.end(), data + offset, data + offset + count);
    offset += count;
    parseAvailable();
  }
}

void RtcmParser::parseAvailable()
{
  while (buffer_.size() >= 3U) {
    const auto preamble = std::find(buffer_.begin(), buffer_.end(), 0xd3U);
    if (preamble == buffer_.end()) {
      buffer_.clear();
      return;
    }
    if (preamble != buffer_.begin()) {
      buffer_.erase(buffer_.begin(), preamble);
      if (buffer_.size() < 3U) {
        return;
      }
    }

    if ((buffer_[1] & 0xfcU) != 0U) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t payload_size =
      (static_cast<std::size_t>(buffer_[1] & 0x03U) << 8U) | buffer_[2];
    const std::size_t frame_size = 3U + payload_size + 3U;
    if (buffer_.size() < frame_size) {
      return;
    }

    const uint32_t expected =
      (static_cast<uint32_t>(buffer_[frame_size - 3U]) << 16U) |
      (static_cast<uint32_t>(buffer_[frame_size - 2U]) << 8U) |
      static_cast<uint32_t>(buffer_[frame_size - 1U]);
    const uint32_t actual = crc24q(buffer_.data(), frame_size - 3U);
    if (actual != expected) {
      ++statistics_.invalid_crc_frames;
      buffer_.erase(buffer_.begin());
      continue;
    }

    ++statistics_.valid_frames;
    if (payload_size >= 2U) {
      const uint16_t message_type = static_cast<uint16_t>(
        (static_cast<uint16_t>(buffer_[3]) << 4U) |
        (static_cast<uint16_t>(buffer_[4]) >> 4U));
      ++statistics_.message_type_counts[message_type];
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }
}

const RtcmStatistics & RtcmParser::statistics() const
{
  return statistics_;
}

std::size_t RtcmParser::bufferedBytes() const
{
  return buffer_.size();
}

}  // namespace vida_ntrip_client
