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

#include "vida_ntrip_client/ntrip_connection.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "vida_ntrip_client/ntrip_protocol.hpp"

namespace vida_ntrip_client
{
namespace
{

class UniqueFd
{
public:
  explicit UniqueFd(int fd = -1)
  : fd_(fd) {}
  ~UniqueFd()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }
  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;
  UniqueFd(UniqueFd && other) noexcept
  : fd_(std::exchange(other.fd_, -1)) {}
  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this != &other) {
      if (fd_ >= 0) {
        ::close(fd_);
      }
      fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
  }
  int get() const {return fd_;}
  int release() {return std::exchange(fd_, -1);}

private:
  int fd_;
};

int remainingMilliseconds(std::chrono::steady_clock::time_point deadline)
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    return 0;
  }
  const auto remaining =
    std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
  return static_cast<int>(std::max<int64_t>(1, remaining));
}

std::string socketError(const char * operation)
{
  return std::string(operation) + ": " + std::strerror(errno);
}

}  // namespace

const char * networkStateName(NetworkState state)
{
  switch (state) {
    case NetworkState::kDisconnected:
      return "disconnected";
    case NetworkState::kConnecting:
      return "connecting";
    case NetworkState::kAuthenticating:
      return "authenticating";
    case NetworkState::kStreaming:
      return "streaming";
    case NetworkState::kBackoff:
      return "backoff";
  }
  return "disconnected";
}

NtripConnection::NtripConnection(
  NtripConnectionConfig config, std::string username, std::string password,
  DataCallback data_callback, GgaProvider gga_provider)
: config_(std::move(config)),
  username_(std::move(username)),
  password_(std::move(password)),
  data_callback_(std::move(data_callback)),
  gga_provider_(std::move(gga_provider)),
  rtcm_parser_(8192U)
{
  int descriptors[2]{};
  if (::pipe2(descriptors, O_CLOEXEC | O_NONBLOCK) != 0) {
    throw std::runtime_error(socketError("could not create shutdown pipe"));
  }
  wake_read_fd_ = descriptors[0];
  wake_write_fd_ = descriptors[1];
}

NtripConnection::~NtripConnection()
{
  stop();
  if (wake_read_fd_ >= 0) {
    ::close(wake_read_fd_);
  }
  if (wake_write_fd_ >= 0) {
    ::close(wake_write_fd_);
  }
}

void NtripConnection::start()
{
  bool expected = false;
  if (!started_.compare_exchange_strong(expected, true)) {
    return;
  }
  stop_requested_.store(false);
  network_thread_ = std::thread(&NtripConnection::run, this);
}

void NtripConnection::stop()
{
  stop_requested_.store(true);
  if (wake_write_fd_ >= 0) {
    const uint8_t byte = 1U;
    const ssize_t ignored = ::write(wake_write_fd_, &byte, sizeof(byte));
    (void)ignored;
  }
  if (network_thread_.joinable()) {
    network_thread_.join();
  }
  setState(NetworkState::kDisconnected, false);
}

ConnectionSnapshot NtripConnection::snapshot() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  ConnectionSnapshot copy = status_;
  if (last_byte_time_.has_value()) {
    copy.last_byte_age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_byte_time_.value()).count();
  }
  return copy;
}

void NtripConnection::setState(NetworkState state, bool connected)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_.state = state;
  status_.connected = connected;
}

void NtripConnection::setError(const std::string & error)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_.last_error = sanitizeForLog(error, username_, password_);
}

void NtripConnection::recordReceived(std::size_t size)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_.bytes_received += size;
  last_byte_time_ = std::chrono::steady_clock::now();
}

void NtripConnection::drainWakePipe()
{
  std::array<uint8_t, 64> buffer{};
  while (::read(wake_read_fd_, buffer.data(), buffer.size()) > 0) {}
}

int NtripConnection::pollSocket(int socket_fd, int16_t events, int timeout_ms)
{
  std::array<pollfd, 2> descriptors{{
    {socket_fd, events, 0},
    {wake_read_fd_, POLLIN, 0},
  }};
  int result;
  do {
    result = ::poll(descriptors.data(), descriptors.size(), timeout_ms);
  } while (result < 0 && errno == EINTR && !stop_requested_.load());
  if (result < 0) {
    setError(socketError("poll failed"));
    return -1;
  }
  if ((descriptors[1].revents & POLLIN) != 0) {
    drainWakePipe();
    return -2;
  }
  if (result == 0) {
    return 0;
  }
  if ((descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 &&
    (descriptors[0].revents & events) == 0)
  {
    return -1;
  }
  return descriptors[0].revents;
}

int NtripConnection::connectSocket()
{
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  addrinfo * addresses = nullptr;
  const std::string service = std::to_string(config_.caster_port);
  const int lookup =
    ::getaddrinfo(config_.caster_host.c_str(), service.c_str(), &hints, &addresses);
  if (lookup != 0) {
    setError(std::string("DNS lookup failed: ") + gai_strerror(lookup));
    return -1;
  }

  std::string last_error = "could not connect to NTRIP caster";
  for (addrinfo * address = addresses; address != nullptr && !stop_requested_.load();
    address = address->ai_next)
  {
    UniqueFd socket_fd(::socket(
        address->ai_family, address->ai_socktype | SOCK_NONBLOCK | SOCK_CLOEXEC,
        address->ai_protocol));
    if (socket_fd.get() < 0) {
      last_error = socketError("socket creation failed");
      continue;
    }
    const int connected = ::connect(socket_fd.get(), address->ai_addr, address->ai_addrlen);
    if (connected == 0) {
      ::freeaddrinfo(addresses);
      return socket_fd.release();
    }
    if (errno != EINPROGRESS) {
      last_error = socketError("connect failed");
      continue;
    }

    const int ready = pollSocket(socket_fd.get(), POLLOUT, config_.connect_timeout_ms);
    if (ready == -2) {
      ::freeaddrinfo(addresses);
      return -1;
    }
    if (ready <= 0) {
      last_error = ready == 0 ? "NTRIP connection timeout" : "NTRIP connection failed";
      continue;
    }
    int error = 0;
    socklen_t error_size = sizeof(error);
    if (::getsockopt(
        socket_fd.get(), SOL_SOCKET, SO_ERROR, &error,
        &error_size) != 0 || error != 0)
    {
      if (error != 0) {
        errno = error;
      }
      last_error = socketError("connect failed");
      continue;
    }
    ::freeaddrinfo(addresses);
    return socket_fd.release();
  }
  ::freeaddrinfo(addresses);
  if (!stop_requested_.load()) {
    setError(last_error);
  }
  return -1;
}

bool NtripConnection::sendAll(
  int socket_fd, const uint8_t * data, std::size_t size, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::size_t sent = 0U;
  while (sent < size && !stop_requested_.load()) {
    const int remaining = remainingMilliseconds(deadline);
    if (remaining == 0) {
      setError("NTRIP socket write timeout");
      return false;
    }
    const int ready = pollSocket(socket_fd, POLLOUT, remaining);
    if (ready <= 0) {
      if (ready == 0) {
        setError("NTRIP socket write timeout");
      }
      return false;
    }
    const ssize_t count = ::send(
      socket_fd, data + sent, size - sent, MSG_NOSIGNAL | MSG_DONTWAIT);
    if (count > 0) {
      sent += static_cast<std::size_t>(count);
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      setError(socketError("NTRIP socket write failed"));
      return false;
    }
  }
  return sent == size;
}

bool NtripConnection::publishBytes(const uint8_t * data, std::size_t size)
{
  if (size == 0U) {
    return true;
  }
  if (config_.enable_rtcm_validation) {
    rtcm_parser_.feed(data, size);
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.rtcm = rtcm_parser_.statistics();
  }
  for (std::size_t offset = 0U; offset < size; offset += config_.publish_chunk_max_bytes) {
    const std::size_t chunk_size = std::min(config_.publish_chunk_max_bytes, size - offset);
    const std::vector<uint8_t> chunk(data + offset, data + offset + chunk_size);
    try {
      data_callback_(chunk);
    } catch (const std::exception & exception) {
      setError(std::string("RTCM publication callback failed: ") + exception.what());
      return false;
    } catch (...) {
      setError("RTCM publication callback failed");
      return false;
    }
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.bytes_published += chunk_size;
  }
  return true;
}

bool NtripConnection::runSession(bool & reached_streaming)
{
  reached_streaming = false;
  setState(NetworkState::kConnecting, false);
  UniqueFd socket_fd(connectSocket());
  if (socket_fd.get() < 0) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_.connections > 0U) {
      ++status_.reconnections;
    }
    ++status_.connections;
    status_.connected = true;
  }

  const std::string request = buildNtripRequest(
    config_.caster_host, config_.caster_port, config_.mountpoint,
    config_.user_agent, username_, password_);
  if (!sendAll(
      socket_fd.get(), reinterpret_cast<const uint8_t *>(request.data()), request.size(),
      config_.header_timeout_ms))
  {
    return false;
  }
  setState(NetworkState::kAuthenticating, true);

  NtripResponseParser parser(config_.max_response_header_bytes);
  const auto header_deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(config_.header_timeout_ms);
  std::array<uint8_t, 4096> buffer{};
  while (!stop_requested_.load()) {
    const int remaining = remainingMilliseconds(header_deadline);
    if (remaining == 0) {
      setError("NTRIP response header timeout");
      return false;
    }
    const int ready = pollSocket(socket_fd.get(), POLLIN, remaining);
    if (ready <= 0) {
      if (ready == 0) {
        setError("NTRIP response header timeout");
      } else if (ready == -1) {
        setError("NTRIP response socket error");
      }
      return false;
    }
    const ssize_t count = ::recv(socket_fd.get(), buffer.data(), buffer.size(), MSG_DONTWAIT);
    if (count == 0) {
      setError("NTRIP caster closed connection before response header");
      return false;
    }
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      setError(socketError("NTRIP response read failed"));
      return false;
    }
    recordReceived(static_cast<std::size_t>(count));
    ResponseResult result = parser.feed(buffer.data(), static_cast<std::size_t>(count));
    if (result.state == ResponseState::kPending) {
      continue;
    }
    if (result.state != ResponseState::kAccepted) {
      setError(result.error);
      return false;
    }
    reached_streaming = true;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_.last_error.clear();
    }
    setState(NetworkState::kStreaming, true);
    if (!publishBytes(result.body.data(), result.body.size())) {
      return false;
    }
    break;
  }

  auto receive_deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(config_.read_timeout_ms);
  auto next_gga = std::chrono::steady_clock::now();
  const auto gga_interval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.gga_interval_s));
  while (!stop_requested_.load()) {
    int timeout = remainingMilliseconds(receive_deadline);
    if (config_.send_gga && gga_provider_) {
      timeout = std::min(timeout, remainingMilliseconds(next_gga));
    }
    const int ready = pollSocket(socket_fd.get(), POLLIN, timeout);
    if (ready == -2) {
      return false;
    }
    if (ready < 0) {
      setError("NTRIP stream socket error");
      return false;
    }

    const auto after_poll = std::chrono::steady_clock::now();
    if (config_.send_gga && gga_provider_ && after_poll >= next_gga) {
      const std::optional<std::string> gga = gga_provider_();
      if (gga.has_value() && !sendAll(
          socket_fd.get(), reinterpret_cast<const uint8_t *>(gga->data()), gga->size(),
          config_.read_timeout_ms))
      {
        return false;
      }
      next_gga = after_poll + gga_interval;
    }
    if (ready == 0) {
      if (after_poll >= receive_deadline) {
        setError("NTRIP stream read timeout");
        return false;
      }
      continue;
    }

    const ssize_t count = ::recv(socket_fd.get(), buffer.data(), buffer.size(), MSG_DONTWAIT);
    if (count == 0) {
      setError("NTRIP caster closed the RTCM stream (EOF)");
      return false;
    }
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      setError(socketError("NTRIP stream read failed"));
      return false;
    }
    recordReceived(static_cast<std::size_t>(count));
    receive_deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(config_.read_timeout_ms);
    if (!publishBytes(buffer.data(), static_cast<std::size_t>(count))) {
      return false;
    }
  }
  return true;
}

bool NtripConnection::waitBackoff(int delay_ms)
{
  pollfd descriptor{wake_read_fd_, POLLIN, 0};
  int result;
  do {
    result = ::poll(&descriptor, 1U, delay_ms);
  } while (result < 0 && errno == EINTR && !stop_requested_.load());
  if (result > 0) {
    drainWakePipe();
  }
  return !stop_requested_.load();
}

void NtripConnection::run()
{
  int delay_ms = config_.reconnect_initial_delay_ms;
  while (!stop_requested_.load()) {
    bool reached_streaming = false;
    runSession(reached_streaming);
    setState(NetworkState::kDisconnected, false);
    if (stop_requested_.load()) {
      break;
    }
    delay_ms = reached_streaming ? config_.reconnect_initial_delay_ms : delay_ms;
    setState(NetworkState::kBackoff, false);
    if (!waitBackoff(delay_ms)) {
      break;
    }
    if (delay_ms >= config_.reconnect_max_delay_ms - delay_ms) {
      delay_ms = config_.reconnect_max_delay_ms;
    } else {
      delay_ms *= 2;
    }
  }
  setState(NetworkState::kDisconnected, false);
}

}  // namespace vida_ntrip_client
