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

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vida_ntrip_client/ntrip_connection.hpp"
#include "vida_ntrip_client/rtcm_parser.hpp"

namespace vida_ntrip_client
{
namespace
{

std::vector<uint8_t> integrationFrame(uint16_t type)
{
  const std::vector<uint8_t> payload = {
    static_cast<uint8_t>(type >> 4U), static_cast<uint8_t>((type & 0x0fU) << 4U),
    0x11, 0x22, 0x33, 0x44};
  std::vector<uint8_t> frame = {
    0xd3, 0x00, static_cast<uint8_t>(payload.size())};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint32_t crc = crc24q(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc >> 16U));
  frame.push_back(static_cast<uint8_t>(crc >> 8U));
  frame.push_back(static_cast<uint8_t>(crc));
  return frame;
}

bool sendBytes(int fd, const uint8_t * data, std::size_t size)
{
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t count = ::send(fd, data + offset, size - offset, MSG_NOSIGNAL);
    if (count <= 0) {
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }
  return true;
}

bool sendText(int fd, const std::string & text)
{
  return sendBytes(fd, reinterpret_cast<const uint8_t *>(text.data()), text.size());
}

std::string receiveRequest(int fd)
{
  std::string request;
  while (request.find("\r\n\r\n") == std::string::npos && request.size() < 8192U) {
    pollfd descriptor{fd, POLLIN, 0};
    if (::poll(&descriptor, 1U, 2000) <= 0) {
      break;
    }
    char buffer[512]{};
    const ssize_t count = ::recv(fd, buffer, sizeof(buffer), 0);
    if (count <= 0) {
      break;
    }
    request.append(buffer, static_cast<std::size_t>(count));
  }
  return request;
}

class FakeCaster
{
public:
  explicit FakeCaster(std::vector<uint8_t> frame)
  : frame_(std::move(frame))
  {
    listener_ = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, IPPROTO_TCP);
    if (listener_ < 0) {
      return;
    }
    int reuse = 1;
    ::setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (::bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      ::listen(listener_, 2) != 0)
    {
      ::close(listener_);
      listener_ = -1;
      return;
    }
    socklen_t address_size = sizeof(address);
    if (::getsockname(
        listener_, reinterpret_cast<sockaddr *>(&address), &address_size) != 0)
    {
      ::close(listener_);
      listener_ = -1;
      return;
    }
    port_ = ntohs(address.sin_port);
    thread_ = std::thread(&FakeCaster::serve, this);
  }

  ~FakeCaster()
  {
    if (listener_ >= 0) {
      ::shutdown(listener_, SHUT_RDWR);
      ::close(listener_);
      listener_ = -1;
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  uint16_t port() const {return port_;}
  bool requestValid() const {return request_valid_.load();}
  bool secondConnectionSeen() const {return second_connection_seen_.load();}

private:
  int acceptOne()
  {
    pollfd descriptor{listener_, POLLIN, 0};
    if (::poll(&descriptor, 1U, 4000) <= 0) {
      return -1;
    }
    return ::accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC);
  }

  bool validRequest(const std::string & request)
  {
    return request.find("GET /MGLA0 HTTP/1.0\r\n") == 0U &&
           request.find("Host: 127.0.0.1:") != std::string::npos &&
           request.find("User-Agent: NTRIP VIDA/1.0\r\n") != std::string::npos &&
           request.find("Authorization: Basic ") != std::string::npos &&
           request.size() >= 4U && request.substr(request.size() - 4U) == "\r\n\r\n";
  }

  void serve()
  {
    const int first = acceptOne();
    if (first < 0) {
      return;
    }
    request_valid_.store(validRequest(receiveRequest(first)));
    sendText(first, "ICY 200");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const std::string end_header = " OK\r\n\r\n";
    std::vector<uint8_t> combined(end_header.begin(), end_header.end());
    const std::size_t first_frame_part = std::min<std::size_t>(4U, frame_.size());
    combined.insert(combined.end(), frame_.begin(), frame_.begin() + first_frame_part);
    sendBytes(first, combined.data(), combined.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const std::size_t middle = std::min(frame_.size(), first_frame_part + 3U);
    sendBytes(first, frame_.data() + first_frame_part, middle - first_frame_part);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    sendBytes(first, frame_.data() + middle, frame_.size() - middle);
    ::shutdown(first, SHUT_RDWR);
    ::close(first);

    const int second = acceptOne();
    if (second < 0) {
      return;
    }
    const std::string second_request = receiveRequest(second);
    request_valid_.store(request_valid_.load() && validRequest(second_request));
    second_connection_seen_.store(true);
    sendText(second, "HTTP/1.1 200 OK\r\nServer: fake\r\n\r\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ::shutdown(second, SHUT_RDWR);
    ::close(second);
  }

  int listener_{-1};
  uint16_t port_{0};
  std::vector<uint8_t> frame_;
  std::atomic<bool> request_valid_{false};
  std::atomic<bool> second_connection_seen_{false};
  std::thread thread_;
};

}  // namespace

TEST(NtripConnection, FakeCasterPreservesBytesReconnectsAndStopsPromptly)
{
  const std::vector<uint8_t> expected = integrationFrame(1074U);
  FakeCaster caster(expected);
  ASSERT_NE(caster.port(), 0U);

  NtripConnectionConfig config;
  config.caster_host = "127.0.0.1";
  config.caster_port = caster.port();
  config.mountpoint = "MGLA0";
  config.connect_timeout_ms = 1000;
  config.header_timeout_ms = 1000;
  config.read_timeout_ms = 5000;
  config.reconnect_initial_delay_ms = 30;
  config.reconnect_max_delay_ms = 100;
  config.max_response_header_bytes = 1024U;
  config.publish_chunk_max_bytes = 3U;

  std::mutex received_mutex;
  std::condition_variable received_condition;
  std::vector<uint8_t> received;
  NtripConnection connection(
    config, "integration-user", "integration-pass",
    [&](const std::vector<uint8_t> & chunk) {
      {
        std::lock_guard<std::mutex> lock(received_mutex);
        received.insert(received.end(), chunk.begin(), chunk.end());
      }
      received_condition.notify_all();
    });
  connection.start();

  {
    std::unique_lock<std::mutex> lock(received_mutex);
    EXPECT_TRUE(
      received_condition.wait_for(
        lock, std::chrono::seconds(4), [&]() {
          return received.size() >= expected.size();
        }));
  }
  const auto reconnect_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (!caster.secondConnectionSeen() && std::chrono::steady_clock::now() < reconnect_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto stop_start = std::chrono::steady_clock::now();
  connection.stop();
  const double stop_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - stop_start).count();

  EXPECT_TRUE(caster.requestValid());
  EXPECT_TRUE(caster.secondConnectionSeen());
  {
    std::lock_guard<std::mutex> lock(received_mutex);
    EXPECT_EQ(received, expected);
  }
  const ConnectionSnapshot snapshot = connection.snapshot();
  EXPECT_GE(snapshot.connections, 2U);
  EXPECT_GE(snapshot.reconnections, 1U);
  EXPECT_EQ(snapshot.bytes_published, expected.size());
  EXPECT_EQ(snapshot.rtcm.valid_frames, 1U);
  EXPECT_LT(stop_seconds, 0.5);
}

}  // namespace vida_ntrip_client
