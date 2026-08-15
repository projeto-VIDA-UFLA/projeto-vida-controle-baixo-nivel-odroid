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

#include "vida_zed_f9r_driver/ubx_parser.hpp"

#include <algorithm>
#include <array>

namespace vida_zed_f9r_driver
{

UbxParser::UbxParser(std::size_t maximum_payload_size)
: maximum_payload_size_(maximum_payload_size)
{
}

std::vector<UbxFrame> UbxParser::feed(const uint8_t * data, std::size_t size)
{
  if (data != nullptr && size != 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<UbxFrame> frames;
  constexpr std::array<uint8_t, 2> kSync{{kUbxSync1, kUbxSync2}};

  while (true) {
    const auto sync = std::search(buffer_.begin(), buffer_.end(), kSync.begin(), kSync.end());
    if (sync == buffer_.end()) {
      const bool keep_sync_prefix = !buffer_.empty() && buffer_.back() == kUbxSync1;
      buffer_.clear();
      if (keep_sync_prefix) {
        buffer_.push_back(kUbxSync1);
      }
      break;
    }

    if (sync != buffer_.begin()) {
      buffer_.erase(buffer_.begin(), sync);
    }
    if (buffer_.size() < 8U) {
      break;
    }

    const std::size_t payload_size = static_cast<std::size_t>(buffer_[4U]) |
      (static_cast<std::size_t>(buffer_[5U]) << 8U);
    if (payload_size > maximum_payload_size_) {
      ++invalid_frames_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_size = payload_size + 8U;
    if (buffer_.size() < frame_size) {
      break;
    }

    const UbxChecksum expected = calculateChecksum(buffer_.data() + 2U, payload_size + 4U);
    const UbxChecksum received{buffer_[frame_size - 2U], buffer_[frame_size - 1U]};
    if (!(expected == received)) {
      ++invalid_frames_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    UbxFrame frame;
    frame.message_class = buffer_[2U];
    frame.message_id = buffer_[3U];
    frame.payload.assign(
      buffer_.begin() + 6, buffer_.begin() + static_cast<std::ptrdiff_t>(6U + payload_size));
    ++valid_frames_;
    if (!isKnownMessage(frame.message_class, frame.message_id)) {
      ++unknown_frames_;
    }
    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }

  return frames;
}

std::vector<UbxFrame> UbxParser::feed(const std::vector<uint8_t> & data)
{
  return feed(data.data(), data.size());
}

void UbxParser::reset()
{
  buffer_.clear();
}

UbxParserStatistics UbxParser::statistics() const
{
  return UbxParserStatistics{
    valid_frames_.load(), invalid_frames_.load(), unknown_frames_.load()};
}

std::size_t UbxParser::bufferedBytes() const
{
  return buffer_.size();
}

}  // namespace vida_zed_f9r_driver
