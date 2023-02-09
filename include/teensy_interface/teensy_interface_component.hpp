// Copyright 2023 ATL Robotics, USA

#ifndef TEENSY_INTERFACE__TEENSY_INTERFACE_COMPONENT_HPP_
#define TEENSY_INTERFACE__TEENSY_INTERFACE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <boost/fusion/adapted/struct.hpp>

#include "udp_server.hpp"

#include <atl_msgs/msg/depth.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <optional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace atl
{

// Forward declarations
class UDPServer;

struct TeensyUdpParams
{
  std::string teensy_ip{"10.250.225.93"};
  uint16_t send_port{1560};
  uint16_t receive_port{1561};
  uint32_t receive_buffer_size{1024};

  void check_correctness() const
  {
    if (teensy_ip.empty()) {
      throw std::invalid_argument("teensy_ip parameter must not be empty.");
    }
    if (receive_buffer_size == 0) {
      throw std::invalid_argument("receive_buffer_size parameter must be > 0.");
    }
  }
};

struct TeensyInterfaceParams
{
  std::size_t n_servos{2};
  TeensyUdpParams udp;

  void check_correctness() const
  {
    if (n_servos < 1) {
      throw std::invalid_argument("n_servos parameter must be > 0.");
    }
    udp.check_correctness();
  }
};

// Component declaration
class TeensyInterfaceComponent : public rclcpp::Node
{
public:
  explicit TeensyInterfaceComponent(const rclcpp::NodeOptions &);

private:
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoystick_;

  // Publishers
  rclcpp::Publisher<atl_msgs::msg::Depth>::SharedPtr pubDepth_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu_;

  // Callbacks
  void subJoystickCb(sensor_msgs::msg::Joy::SharedPtr && msg);
  void udpCb(const UDPServer::UDPMsg & msg);

  // State Variables
  std::unique_ptr<UDPServer> udp_;
  uint64_t t0_;
  std::size_t iter_ = 0;
  bool sync_ = true;
  TeensyInterfaceParams prm_;

  uint32_t task_ = 0;
  uint32_t ctrlMode_ = 0;
  std::mutex msgMtx_;

};

}  // namespace atl

// cppcheck-suppress unknownMacro
BOOST_FUSION_ADAPT_STRUCT(
  atl::TeensyUdpParams,
  teensy_ip,
  send_port,
  receive_port,
  receive_buffer_size
)

// cppcheck-suppress unknownMacro
BOOST_FUSION_ADAPT_STRUCT(
  atl::TeensyInterfaceParams,
  n_servos,
  udp
)

#endif  // TEENSY_INTERFACE__TEENSY_INTERFACE_COMPONENT_HPP_
