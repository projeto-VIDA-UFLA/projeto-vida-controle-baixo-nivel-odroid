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

#ifndef VIDA_NTRIP_CLIENT__RTCM_PARSER_HPP_
#define VIDA_NTRIP_CLIENT__RTCM_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace vida_ntrip_client
{

uint32_t crc24q(const uint8_t * data, std::size_t size);

struct RtcmStatistics
{
  uint64_t valid_frames{0};
  uint64_t invalid_crc_frames{0};
  std::map<uint16_t, uint64_t> message_type_counts;
};

class RtcmParser
{
public:
  explicit RtcmParser(std::size_t max_buffer_bytes = 8192U);

  void feed(const uint8_t * data, std::size_t size);
  void feed(const std::vector<uint8_t> & data);
  const RtcmStatistics & statistics() const;
  std::size_t bufferedBytes() const;

private:
  void parseAvailable();

  std::size_t max_buffer_bytes_;
  std::vector<uint8_t> buffer_;
  RtcmStatistics statistics_;
};

}  // namespace vida_ntrip_client

#endif  // VIDA_NTRIP_CLIENT__RTCM_PARSER_HPP_
