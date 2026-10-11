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

#include "vida_ntrip_client/ntrip_protocol.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace vida_ntrip_client
{
namespace
{

bool containsLineBreak(std::string_view value)
{
  return value.find('\r') != std::string_view::npos ||
         value.find('\n') != std::string_view::npos;
}

std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::toupper(c));
    });
  return value;
}

std::string formatCoordinate(double value, bool latitude, char & hemisphere)
{
  hemisphere = latitude ? (value >= 0.0 ? 'N' : 'S') : (value >= 0.0 ? 'E' : 'W');
  const double absolute = std::abs(value);
  int degrees = static_cast<int>(std::floor(absolute));
  double minutes = std::round(
    (absolute - static_cast<double>(degrees)) * 60.0 * 100000.0) / 100000.0;
  if (minutes >= 60.0) {
    ++degrees;
    minutes = 0.0;
  }

  std::ostringstream output;
  output << std::setfill('0') << std::setw(latitude ? 2 : 3) << degrees
         << std::fixed << std::setprecision(5) << std::setw(8) << minutes;
  return output.str();
}

}  // namespace

std::string base64Encode(std::string_view input)
{
  static constexpr char kAlphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve(((input.size() + 2U) / 3U) * 4U);

  for (std::size_t offset = 0; offset < input.size(); offset += 3U) {
    const uint32_t first = static_cast<unsigned char>(input[offset]);
    const uint32_t second = offset + 1U < input.size() ?
      static_cast<unsigned char>(input[offset + 1U]) : 0U;
    const uint32_t third = offset + 2U < input.size() ?
      static_cast<unsigned char>(input[offset + 2U]) : 0U;
    const uint32_t group = (first << 16U) | (second << 8U) | third;

    output.push_back(kAlphabet[(group >> 18U) & 0x3fU]);
    output.push_back(kAlphabet[(group >> 12U) & 0x3fU]);
    output.push_back(offset + 1U < input.size() ? kAlphabet[(group >> 6U) & 0x3fU] : '=');
    output.push_back(offset + 2U < input.size() ? kAlphabet[group & 0x3fU] : '=');
  }
  return output;
}

std::string buildNtripRequest(
  const std::string & host, uint16_t port, const std::string & mountpoint,
  const std::string & user_agent, const std::string & username,
  const std::string & password)
{
  if (host.empty() || mountpoint.empty() || user_agent.empty() ||
    containsLineBreak(host) || containsLineBreak(mountpoint) ||
    containsLineBreak(user_agent))
  {
    throw std::invalid_argument("invalid NTRIP request field");
  }

  const std::string path = mountpoint.front() == '/' ? mountpoint : "/" + mountpoint;
  std::ostringstream request;
  request << "GET " << path << " HTTP/1.0\r\n"
          << "Host: " << host << ':' << port << "\r\n"
          << "User-Agent: " << user_agent << "\r\n"
          << "Authorization: Basic " << base64Encode(username + ":" + password) << "\r\n"
          << "Accept: */*\r\n"
          << "Connection: close\r\n\r\n";
  return request.str();
}

std::string sanitizeForLog(
  std::string_view input, std::string_view username, std::string_view password)
{
  std::istringstream stream{std::string(input)};
  std::string line;
  std::string output;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (uppercase(line).find("AUTHORIZATION:") != std::string::npos) {
      continue;
    }
    for (const auto secret : {username, password}) {
      if (secret.empty()) {
        continue;
      }
      std::size_t position = 0;
      while ((position = line.find(secret, position)) != std::string::npos) {
        line.replace(position, secret.size(), "[redacted]");
        position += 10U;
      }
    }
    if (!output.empty()) {
      output.push_back(' ');
    }
    output += line;
  }
  return output;
}

std::optional<std::string> buildGgaSentence(
  double latitude_deg, double longitude_deg, std::string_view utc_hhmmss)
{
  if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
    latitude_deg < -90.0 || latitude_deg > 90.0 ||
    longitude_deg < -180.0 || longitude_deg > 180.0 ||
    utc_hhmmss.size() != 6U ||
    !std::all_of(
      utc_hhmmss.begin(), utc_hhmmss.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
      }))
  {
    return std::nullopt;
  }

  char latitude_hemisphere = 'N';
  char longitude_hemisphere = 'E';
  const std::string latitude = formatCoordinate(latitude_deg, true, latitude_hemisphere);
  const std::string longitude = formatCoordinate(longitude_deg, false, longitude_hemisphere);

  // Empty altitude/geoid/DGPS fields are preferable to fabricating measurements.
  std::ostringstream payload;
  payload << "GPGGA," << utc_hhmmss << ',' << latitude << ',' << latitude_hemisphere << ','
          << longitude << ',' << longitude_hemisphere << ",1,,,,,,,,";
  const std::string payload_text = payload.str();
  uint8_t checksum = 0;
  for (const unsigned char character : payload_text) {
    checksum ^= character;
  }

  char checksum_text[3]{};
  std::snprintf(checksum_text, sizeof(checksum_text), "%02X", checksum);
  return "$" + payload_text + "*" + checksum_text + "\r\n";
}

NtripResponseParser::NtripResponseParser(std::size_t max_header_bytes)
: max_header_bytes_(max_header_bytes)
{
  if (max_header_bytes_ < 4U) {
    throw std::invalid_argument("max response header must be at least 4 bytes");
  }
}

ResponseResult NtripResponseParser::feed(const std::vector<uint8_t> & data)
{
  return feed(data.data(), data.size());
}

ResponseResult NtripResponseParser::feed(const uint8_t * data, std::size_t size)
{
  if (finished_) {
    return {ResponseState::kRejected, {}, "response header parser already finished"};
  }
  if (data == nullptr && size != 0U) {
    return {ResponseState::kRejected, {}, "invalid response buffer"};
  }

  if (size != 0U) {
    header_.insert(header_.end(), data, data + size);
  }
  const std::array<uint8_t, 4> delimiter{{'\r', '\n', '\r', '\n'}};
  const auto delimiter_position = std::search(
    header_.begin(), header_.end(), delimiter.begin(), delimiter.end());
  if (delimiter_position == header_.end()) {
    if (header_.size() > max_header_bytes_) {
      finished_ = true;
      return {ResponseState::kHeaderTooLarge, {}, "NTRIP response header exceeds configured limit"};
    }
    return {};
  }

  const std::size_t header_size =
    static_cast<std::size_t>(std::distance(header_.begin(), delimiter_position)) + delimiter.size();
  if (header_size > max_header_bytes_) {
    finished_ = true;
    return {ResponseState::kHeaderTooLarge, {}, "NTRIP response header exceeds configured limit"};
  }
  finished_ = true;

  const std::string response_header(header_.begin(), delimiter_position);
  const std::size_t line_end = response_header.find("\r\n");
  const std::string status_line = response_header.substr(0, line_end);
  const std::string upper_status = uppercase(status_line);
  const std::string upper_header = uppercase(response_header);

  ResponseResult result;
  result.body.assign(header_.begin() + static_cast<std::ptrdiff_t>(header_size), header_.end());
  const bool accepted =
    upper_status.rfind("ICY 200", 0) == 0 ||
    upper_status.rfind("HTTP/1.0 200", 0) == 0 ||
    upper_status.rfind("HTTP/1.1 200", 0) == 0;
  const bool sourcetable =
    upper_status.rfind("SOURCETABLE", 0) == 0 ||
    upper_header.find("CONTENT-TYPE: GNSS/SOURCETABLE") != std::string::npos;

  if (sourcetable) {
    result.state = ResponseState::kRejected;
    result.body.clear();
    result.error = "caster returned a sourcetable instead of an RTCM stream; check mountpoint";
  } else if (accepted) {
    result.state = ResponseState::kAccepted;
  } else {
    result.state = ResponseState::kRejected;
    result.body.clear();
    if (upper_status.find(" 401") != std::string::npos) {
      result.error = "NTRIP authentication failed (401)";
    } else if (upper_status.find(" 403") != std::string::npos) {
      result.error = "NTRIP access forbidden (403)";
    } else if (upper_status.find(" 404") != std::string::npos) {
      result.error = "NTRIP mountpoint not found (404)";
    } else {
      result.error = "NTRIP caster rejected response: " + sanitizeForLog(status_line);
    }
  }
  return result;
}

}  // namespace vida_ntrip_client
