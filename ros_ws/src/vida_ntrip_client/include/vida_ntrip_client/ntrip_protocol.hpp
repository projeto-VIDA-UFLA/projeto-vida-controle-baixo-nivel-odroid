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

#ifndef VIDA_NTRIP_CLIENT__NTRIP_PROTOCOL_HPP_
#define VIDA_NTRIP_CLIENT__NTRIP_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace vida_ntrip_client
{

std::string base64Encode(std::string_view input);

std::string buildNtripRequest(
  const std::string & host, uint16_t port, const std::string & mountpoint,
  const std::string & user_agent, const std::string & username,
  const std::string & password);

std::string sanitizeForLog(
  std::string_view input, std::string_view username = {},
  std::string_view password = {});

std::optional<std::string> buildGgaSentence(
  double latitude_deg, double longitude_deg, std::string_view utc_hhmmss);

enum class ResponseState
{
  kPending,
  kAccepted,
  kRejected,
  kHeaderTooLarge,
};

struct ResponseResult
{
  ResponseState state{ResponseState::kPending};
  std::vector<uint8_t> body;
  std::string error;
};

class NtripResponseParser
{
public:
  explicit NtripResponseParser(std::size_t max_header_bytes);

  ResponseResult feed(const uint8_t * data, std::size_t size);
  ResponseResult feed(const std::vector<uint8_t> & data);

private:
  std::size_t max_header_bytes_;
  std::vector<uint8_t> header_;
  bool finished_{false};
};

}  // namespace vida_ntrip_client

#endif  // VIDA_NTRIP_CLIENT__NTRIP_PROTOCOL_HPP_
