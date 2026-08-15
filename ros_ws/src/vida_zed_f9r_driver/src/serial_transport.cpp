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

#include "vida_zed_f9r_driver/serial_transport.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <optional>
#include <utility>

namespace vida_zed_f9r_driver
{
namespace
{

std::optional<speed_t> termiosSpeed(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return std::nullopt;
  }
}

std::string systemError(const std::string & operation)
{
  return operation + ": " + std::strerror(errno);
}

}  // namespace

SerialTransport::SerialTransport(
  std::string device, int baudrate, int reconnect_interval_ms,
  DataCallback data_callback, StateCallback state_callback)
: device_(std::move(device)),
  baudrate_(baudrate),
  reconnect_interval_ms_(reconnect_interval_ms),
  data_callback_(std::move(data_callback)),
  state_callback_(std::move(state_callback))
{
}

SerialTransport::~SerialTransport()
{
  stop();
}

void SerialTransport::start()
{
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    return;
  }
  read_thread_ = std::thread(&SerialTransport::run, this);
}

void SerialTransport::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  wait_condition_.notify_all();
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  closeDevice();
}

bool SerialTransport::writeBytes(
  const uint8_t * data, std::size_t size, std::string * error)
{
  if (size == 0U) {
    return true;
  }
  if (data == nullptr) {
    if (error != nullptr) {
      *error = "null data pointer";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(write_mutex_);
  const int descriptor = file_descriptor_.load();
  if (descriptor < 0) {
    if (error != nullptr) {
      *error = "receiver is disconnected";
    }
    return false;
  }

  std::size_t written = 0U;
  while (written < size) {
    const ssize_t result = ::write(descriptor, data + written, size - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd poll_descriptor{descriptor, POLLOUT, 0};
      const int poll_result = ::poll(&poll_descriptor, 1, 1000);
      if (poll_result > 0 && (poll_descriptor.revents & POLLOUT) != 0) {
        continue;
      }
      if (error != nullptr) {
        *error = poll_result == 0 ? "serial write timed out" : systemError("poll(POLLOUT)");
      }
      return false;
    }
    if (error != nullptr) {
      *error = result == 0 ? "serial write returned zero" : systemError("write");
    }
    return false;
  }
  return true;
}

bool SerialTransport::writeBytes(const std::vector<uint8_t> & data, std::string * error)
{
  return writeBytes(data.data(), data.size(), error);
}

bool SerialTransport::connected() const
{
  return connected_.load();
}

void SerialTransport::run()
{
  while (running_.load()) {
    std::string detail;
    const int descriptor = openDevice(detail);
    if (descriptor < 0) {
      try {
        state_callback_(false, detail);
      } catch (...) {
        // User callbacks must never escape the transport thread.
      }
      if (waitForReconnect()) {
        break;
      }
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      file_descriptor_.store(descriptor);
      connected_.store(true);
    }
    try {
      state_callback_(true, device_);
    } catch (...) {
      // Data transport remains usable even if a state callback fails.
    }

    detail = "receiver disconnected";
    std::array<uint8_t, 4096> read_buffer{};
    while (running_.load()) {
      pollfd poll_descriptor{descriptor, POLLIN, 0};
      const int poll_result = ::poll(&poll_descriptor, 1, 250);
      if (poll_result == 0) {
        continue;
      }
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        detail = systemError("poll(POLLIN)");
        break;
      }
      if ((poll_descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        detail = "serial poll reported disconnect/error";
        break;
      }
      if ((poll_descriptor.revents & POLLIN) == 0) {
        continue;
      }

      const ssize_t count = ::read(descriptor, read_buffer.data(), read_buffer.size());
      if (count > 0) {
        try {
          data_callback_(read_buffer.data(), static_cast<std::size_t>(count));
        } catch (const std::exception & exception) {
          detail = std::string("data callback failed: ") + exception.what();
          break;
        } catch (...) {
          detail = "data callback failed with an unknown exception";
          break;
        }
      } else if (count == 0) {
        detail = "serial read reached end of file";
        break;
      } else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
        detail = systemError("read");
        break;
      }
    }

    closeDevice();
    if (running_.load()) {
      try {
        state_callback_(false, detail);
      } catch (...) {
        // User callbacks must never escape the transport thread.
      }
      if (waitForReconnect()) {
        break;
      }
    }
  }
  closeDevice();
}

int SerialTransport::openDevice(std::string & error) const
{
  const int descriptor = ::open(
    device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (descriptor < 0) {
    error = systemError("open(" + device_ + ")");
    return -1;
  }

  termios settings{};
  if (::tcgetattr(descriptor, &settings) != 0) {
    error = systemError("tcgetattr(" + device_ + ")");
    ::close(descriptor);
    return -1;
  }
  const std::optional<speed_t> speed = termiosSpeed(baudrate_);
  if (!speed.has_value()) {
    error = "unsupported baudrate: " + std::to_string(baudrate_);
    ::close(descriptor);
    return -1;
  }

  ::cfmakeraw(&settings);
  settings.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
  settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (::cfsetispeed(&settings, speed.value()) != 0 ||
    ::cfsetospeed(&settings, speed.value()) != 0 ||
    ::tcsetattr(descriptor, TCSANOW, &settings) != 0)
  {
    error = systemError("configure(" + device_ + ")");
    ::close(descriptor);
    return -1;
  }
  return descriptor;
}

void SerialTransport::closeDevice()
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  const int descriptor = file_descriptor_.exchange(-1);
  connected_.store(false);
  if (descriptor >= 0) {
    ::close(descriptor);
  }
}

bool SerialTransport::waitForReconnect()
{
  std::unique_lock<std::mutex> lock(wait_mutex_);
  return wait_condition_.wait_for(
    lock, std::chrono::milliseconds(reconnect_interval_ms_),
    [this]() {return !running_.load();});
}

}  // namespace vida_zed_f9r_driver
