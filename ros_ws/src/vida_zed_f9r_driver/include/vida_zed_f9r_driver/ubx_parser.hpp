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

#ifndef VIDA_ZED_F9R_DRIVER__UBX_PARSER_HPP_
#define VIDA_ZED_F9R_DRIVER__UBX_PARSER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "vida_zed_f9r_driver/ubx_protocol.hpp"

namespace vida_zed_f9r_driver
{

struct UbxParserStatistics
{
  uint64_t valid_frames{0};
  uint64_t invalid_frames{0};
  uint64_t unknown_frames{0};
};

class UbxParser
{
public:
  explicit UbxParser(std::size_t maximum_payload_size = 8192U);

  std::vector<UbxFrame> feed(const uint8_t * data, std::size_t size);
  std::vector<UbxFrame> feed(const std::vector<uint8_t> & data);
  void reset();
  UbxParserStatistics statistics() const;
  std::size_t bufferedBytes() const;

private:
  std::vector<uint8_t> buffer_;
  std::size_t maximum_payload_size_;
  std::atomic<uint64_t> valid_frames_{0};
  std::atomic<uint64_t> invalid_frames_{0};
  std::atomic<uint64_t> unknown_frames_{0};
};

}  // namespace vida_zed_f9r_driver

#endif  // VIDA_ZED_F9R_DRIVER__UBX_PARSER_HPP_
