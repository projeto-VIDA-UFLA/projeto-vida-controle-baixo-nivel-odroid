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

#ifndef VIDA_ZED_F9R_DRIVER__SERIAL_TRANSPORT_HPP_
#define VIDA_ZED_F9R_DRIVER__SERIAL_TRANSPORT_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace vida_zed_f9r_driver
{

class SerialTransport
{
public:
  using DataCallback = std::function<void (const uint8_t *, std::size_t)>;
  using StateCallback = std::function<void (bool, const std::string &)>;

  SerialTransport(
    std::string device, int baudrate, int reconnect_interval_ms,
    DataCallback data_callback, StateCallback state_callback);
  ~SerialTransport();

  SerialTransport(const SerialTransport &) = delete;
  SerialTransport & operator=(const SerialTransport &) = delete;

  void start();
  void stop();
  bool writeBytes(const uint8_t * data, std::size_t size, std::string * error = nullptr);
  bool writeBytes(const std::vector<uint8_t> & data, std::string * error = nullptr);
  bool connected() const;

private:
  void run();
  int openDevice(std::string & error) const;
  void closeDevice();
  bool waitForReconnect();

  std::string device_;
  int baudrate_;
  int reconnect_interval_ms_;
  DataCallback data_callback_;
  StateCallback state_callback_;

  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<int> file_descriptor_{-1};
  std::thread read_thread_;
  std::mutex write_mutex_;
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
};

}  // namespace vida_zed_f9r_driver

#endif  // VIDA_ZED_F9R_DRIVER__SERIAL_TRANSPORT_HPP_
