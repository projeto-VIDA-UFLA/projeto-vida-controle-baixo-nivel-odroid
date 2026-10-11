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

#ifndef VIDA_NTRIP_CLIENT__NTRIP_CONNECTION_HPP_
#define VIDA_NTRIP_CLIENT__NTRIP_CONNECTION_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "vida_ntrip_client/rtcm_parser.hpp"

namespace vida_ntrip_client
{

struct NtripConnectionConfig
{
  std::string caster_host{"170.84.40.52"};
  uint16_t caster_port{2101};
  std::string mountpoint{"MGLA0"};
  std::string user_agent{"NTRIP VIDA/1.0"};
  int connect_timeout_ms{5000};
  int header_timeout_ms{5000};
  int read_timeout_ms{15000};
  int reconnect_initial_delay_ms{1000};
  int reconnect_max_delay_ms{30000};
  std::size_t max_response_header_bytes{8192U};
  std::size_t publish_chunk_max_bytes{4096U};
  bool enable_rtcm_validation{true};
  bool send_gga{false};
  double gga_interval_s{10.0};
};

enum class NetworkState
{
  kDisconnected,
  kConnecting,
  kAuthenticating,
  kStreaming,
  kBackoff,
};

const char * networkStateName(NetworkState state);

struct ConnectionSnapshot
{
  NetworkState state{NetworkState::kDisconnected};
  bool connected{false};
  double last_byte_age_s{-1.0};
  uint64_t bytes_received{0};
  uint64_t bytes_published{0};
  uint64_t connections{0};
  uint64_t reconnections{0};
  std::string last_error;
  RtcmStatistics rtcm;
};

class NtripConnection
{
public:
  using DataCallback = std::function<void (const std::vector<uint8_t> &)>;
  using GgaProvider = std::function<std::optional<std::string>()>;

  NtripConnection(
    NtripConnectionConfig config, std::string username, std::string password,
    DataCallback data_callback, GgaProvider gga_provider = {});
  ~NtripConnection();

  NtripConnection(const NtripConnection &) = delete;
  NtripConnection & operator=(const NtripConnection &) = delete;

  void start();
  void stop();
  ConnectionSnapshot snapshot() const;

private:
  void run();
  bool runSession(bool & reached_streaming);
  int connectSocket();
  bool sendAll(int socket_fd, const uint8_t * data, std::size_t size, int timeout_ms);
  bool publishBytes(const uint8_t * data, std::size_t size);
  int pollSocket(int socket_fd, int16_t events, int timeout_ms);
  bool waitBackoff(int delay_ms);
  void setState(NetworkState state, bool connected);
  void setError(const std::string & error);
  void recordReceived(std::size_t size);
  void drainWakePipe();

  NtripConnectionConfig config_;
  std::string username_;
  std::string password_;
  DataCallback data_callback_;
  GgaProvider gga_provider_;
  RtcmParser rtcm_parser_;

  mutable std::mutex status_mutex_;
  ConnectionSnapshot status_;
  std::optional<std::chrono::steady_clock::time_point> last_byte_time_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> started_{false};
  std::thread network_thread_;
  int wake_read_fd_{-1};
  int wake_write_fd_{-1};
};

}  // namespace vida_ntrip_client

#endif  // VIDA_NTRIP_CLIENT__NTRIP_CONNECTION_HPP_
