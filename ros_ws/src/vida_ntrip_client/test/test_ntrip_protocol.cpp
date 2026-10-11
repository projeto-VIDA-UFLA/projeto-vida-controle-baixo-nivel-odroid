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

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "vida_ntrip_client/ntrip_protocol.hpp"

namespace vida_ntrip_client
{

TEST(NtripProtocol, EncodesBase64)
{
  EXPECT_EQ(base64Encode(""), "");
  EXPECT_EQ(base64Encode("f"), "Zg==");
  EXPECT_EQ(base64Encode("fo"), "Zm8=");
  EXPECT_EQ(base64Encode("foo"), "Zm9v");
  EXPECT_EQ(base64Encode("Aladdin:open sesame"), "QWxhZGRpbjpvcGVuIHNlc2FtZQ==");
}

TEST(NtripProtocol, BuildsNtripV1Request)
{
  const std::string request = buildNtripRequest(
    "127.0.0.1", 2101, "MGLA0", "NTRIP VIDA/1.0", "unit-user", "unit-pass");
  EXPECT_EQ(request.find("GET /MGLA0 HTTP/1.0\r\n"), 0U);
  EXPECT_NE(request.find("Host: 127.0.0.1:2101\r\n"), std::string::npos);
  EXPECT_NE(request.find("User-Agent: NTRIP VIDA/1.0\r\n"), std::string::npos);
  EXPECT_NE(
    request.find("Authorization: Basic dW5pdC11c2VyOnVuaXQtcGFzcw==\r\n"),
    std::string::npos);
  EXPECT_EQ(request.substr(request.size() - 4U), "\r\n\r\n");
}

TEST(NtripProtocol, RejectsHeaderInjection)
{
  EXPECT_THROW(
    buildNtripRequest("caster", 2101, "MGLA0\r\nInjected: yes", "agent", "user", "pass"),
    std::invalid_argument);
}

TEST(NtripProtocol, SanitizesSecretsAndAuthorization)
{
  const std::string sanitized = sanitizeForLog(
    "request failed\r\nAuthorization: Basic abc\r\nuser=sentinel-user pass=sentinel-pass",
    "sentinel-user", "sentinel-pass");
  EXPECT_EQ(sanitized.find("Authorization"), std::string::npos);
  EXPECT_EQ(sanitized.find("sentinel-user"), std::string::npos);
  EXPECT_EQ(sanitized.find("sentinel-pass"), std::string::npos);
}

TEST(NtripResponse, AcceptsIcyAndHttpSuccess)
{
  for (const std::string header : {
      "ICY 200 OK\r\n\r\n", "HTTP/1.0 200 OK\r\n\r\n",
      "HTTP/1.1 200 OK\r\nServer: fake\r\n\r\n"})
  {
    NtripResponseParser parser(1024U);
    const std::vector<uint8_t> bytes(header.begin(), header.end());
    EXPECT_EQ(parser.feed(bytes).state, ResponseState::kAccepted);
  }
}

TEST(NtripResponse, DiagnosesHttpFailures)
{
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"HTTP/1.1 401 Unauthorized\r\n\r\n", "401"},
    {"HTTP/1.1 403 Forbidden\r\n\r\n", "403"},
    {"HTTP/1.1 404 Not Found\r\n\r\n", "404"},
  };
  for (const auto & item : cases) {
    NtripResponseParser parser(1024U);
    const std::vector<uint8_t> bytes(item.first.begin(), item.first.end());
    const ResponseResult result = parser.feed(bytes);
    EXPECT_EQ(result.state, ResponseState::kRejected);
    EXPECT_NE(result.error.find(item.second), std::string::npos);
  }
}

TEST(NtripResponse, RejectsSourcetable)
{
  NtripResponseParser parser(1024U);
  const std::string response = "SOURCETABLE 200 OK\r\nContent-Type: text/plain\r\n\r\n";
  const ResponseResult result = parser.feed(
    reinterpret_cast<const uint8_t *>(response.data()), response.size());
  EXPECT_EQ(result.state, ResponseState::kRejected);
  EXPECT_NE(result.error.find("sourcetable"), std::string::npos);
}

TEST(NtripResponse, HandlesFragmentedHeader)
{
  NtripResponseParser parser(1024U);
  const std::string first = "ICY 20";
  const std::string second = "0 OK\r\nServer: fake\r\n";
  const std::string third = "\r\n";
  EXPECT_EQ(
    parser.feed(reinterpret_cast<const uint8_t *>(first.data()), first.size()).state,
    ResponseState::kPending);
  EXPECT_EQ(
    parser.feed(reinterpret_cast<const uint8_t *>(second.data()), second.size()).state,
    ResponseState::kPending);
  EXPECT_EQ(
    parser.feed(reinterpret_cast<const uint8_t *>(third.data()), third.size()).state,
    ResponseState::kAccepted);
}

TEST(NtripResponse, PreservesBodyFromHeaderBuffer)
{
  NtripResponseParser parser(1024U);
  const std::vector<uint8_t> response = {
    'I', 'C', 'Y', ' ', '2', '0', '0', ' ', 'O', 'K', '\r', '\n', '\r', '\n',
    0xd3, 0x00, 0x01, 0xaa};
  const ResponseResult result = parser.feed(response);
  EXPECT_EQ(result.state, ResponseState::kAccepted);
  EXPECT_EQ(result.body, (std::vector<uint8_t>{0xd3, 0x00, 0x01, 0xaa}));
}

TEST(NtripResponse, EnforcesHeaderLimit)
{
  NtripResponseParser parser(16U);
  const std::string oversized(17U, 'x');
  const ResponseResult result = parser.feed(
    reinterpret_cast<const uint8_t *>(oversized.data()), oversized.size());
  EXPECT_EQ(result.state, ResponseState::kHeaderTooLarge);
}

TEST(Gga, FormatsCoordinatesHemispheresAndChecksum)
{
  const auto sentence = buildGgaSentence(-21.245, -44.999, "123519");
  ASSERT_TRUE(sentence.has_value());
  EXPECT_NE(sentence->find("$GPGGA,123519,2114.70000,S,04459.94000,W,1,"), std::string::npos);
  const std::size_t star = sentence->find('*');
  ASSERT_NE(star, std::string::npos);
  uint8_t checksum = 0U;
  for (std::size_t index = 1U; index < star; ++index) {
    checksum ^= static_cast<uint8_t>((*sentence)[index]);
  }
  char expected[3]{};
  std::snprintf(expected, sizeof(expected), "%02X", checksum);
  EXPECT_EQ(sentence->substr(star + 1U, 2U), expected);
  EXPECT_EQ(sentence->substr(sentence->size() - 2U), "\r\n");
}

TEST(Gga, RejectsInvalidFix)
{
  EXPECT_FALSE(buildGgaSentence(91.0, 0.0, "123519").has_value());
  EXPECT_FALSE(buildGgaSentence(0.0, 181.0, "123519").has_value());
  EXPECT_FALSE(buildGgaSentence(0.0, 0.0, "notime").has_value());
}

}  // namespace vida_ntrip_client
