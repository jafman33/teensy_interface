// Copyright 2023 ATL Robotics, USA

#ifndef TEENSY_INTERFACE__UDP_SERVER_HPP_
#define TEENSY_INTERFACE__UDP_SERVER_HPP_

#include <fmt/core.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cerrno>

#include <cstdint>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <cstring>

#include <chrono>
#include <thread>
#include <mutex>
#include <random>
#include <utility>
#include <functional>
#include <atomic>

namespace atl
{

class UDPServer
{
public:
  using clock_t = std::chrono::high_resolution_clock;
  using timePoint_t = std::chrono::time_point<clock_t>;

  struct UDPMsg
  {
    size_t seq;
    timePoint_t time;
    std::vector<uint8_t> data;
  };

  UDPServer() = default;
  UDPServer(const UDPServer & rhs) = delete;
  UDPServer(UDPServer && rhs) = delete;
  UDPServer & operator=(const UDPServer & rhs) = delete;
  UDPServer & operator=(UDPServer && rhs) = delete;
  ~UDPServer();

  explicit UDPServer(bool verbose);

  void init();

  void init(
    uint32_t recvMaxSize,
    uint16_t sendPort = 1560,
    std::string_view sendAddr = "",
    uint16_t recvPort = 1561,
    std::string_view recvAddr = "");

  UDPMsg getMsg();

  void sendMsg(const std::vector<uint8_t> & msg) const;
  void sendMsg(
    const std::vector<uint8_t> & msg,
    uint16_t sendPort,
    const std::string & sendAddr) const;

  template<typename T>
  void subscribe(T && cb)
  {
    if (verbose_) {
      fmt::print("Subscribed to UDPServer.\n");
    }
    cb_ = std::forward<T>(cb);
    subscribed_ = true;
  }

private:
  void udpReceiveThreadFun();

  bool initialized_ = false;
  bool verbose_ = false;
  std::atomic<bool> isRunning_ = false;
  bool subscribed_ = false;

  std::size_t seq_ = 0;
  uint32_t recvMsgSize_ = 0;
  std::vector<uint8_t> recvBuff_{};
  timePoint_t recvTimePoint_{};
  uint16_t sendPort_{};
  std::string sendAddr_{};
  uint16_t recvPort_{};
  std::string recvAddr_{};

  int32_t socket_recv_{};
  int32_t socket_send_{};
  sockaddr_in servaddr_recv_ {};
  sockaddr_in servaddr_send_ {};

  std::thread recvThread_{};
  std::mutex recvMutex_{};

  UDPMsg currMsg_{};

  std::function<void(const UDPMsg & /*msg*/)> cb_;
};

}  // namespace atl
#endif  // ATL_LIBRARY__UDP_SERVER_HPP_
