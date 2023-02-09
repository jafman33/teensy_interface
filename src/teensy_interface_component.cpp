// Copyright 2023 ATL Robotics, USA
#include "teensy_interface/teensy_interface_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "udp_server.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <exception>
#include <cerrno>
#include <utility>

using std::string;

constexpr static std::size_t nServos = 2;

namespace atl
{

// ///////////////////
// Interface Component
// ///////////////////
TeensyInterfaceComponent::TeensyInterfaceComponent(const rclcpp::NodeOptions & options)
: Node("teensy_interface", options),
  udp_(std::make_unique<atl::UDPServer>(true))
{
  // initParamsInNode(*this, prm_, "", false, true);

  // if (prm_.n_servos > nServos || prm_.n_servos < 1) {
  //   throw std::runtime_error("Unsupported number of servos");
  // }

  // Create the subscriptions
  rclcpp::SensorDataQoS inputQoS;
  inputQoS.keep_last(1);
  subJoystick_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", inputQoS,
    [this](sensor_msgs::msg::Joy::SharedPtr msg) {
      subJoystickCb(std::move(msg));
    });

  // Create the publishers
  pubDepth_ = create_publisher<atl_msgs::msg::Depth>(
    "depth", rclcpp::SystemDefaultsQoS());
  pubImu_ = create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SystemDefaultsQoS());

    // set up UDP communication
  udp_->init(
    prm_.udp.receive_buffer_size,
    prm_.udp.send_port,
    prm_.udp.teensy_ip,
    prm_.udp.receive_port
  );

  udp_->subscribe(
    [this](const UDPServer::UDPMsg & msg) {
      udpCb(msg);
    });

  RCLCPP_INFO(get_logger(), "Teensy Interface Node started");
}


//////////////////
// UDP Msg Receive
//////////////////
void TeensyInterfaceComponent::udpCb(const UDPServer::UDPMsg & msg)
{
  // Lin_acc(3) + Ang_vel(3) + Quat(4) + depth(1) + temp(1)
  constexpr std::size_t msgLen = (3+3+4+1+1) * 4;

  if (msg.data.size() != msgLen) {
    RCLCPP_ERROR(
      get_logger(), "Teensy UDP message configured to be of length %lu, received %lu.",
      msgLen, msg.data.size());
    throw std::runtime_error("Teensy UDP message has incorrect size.");
  }

  std::size_t oft = 0;
  const auto tNow = now();

  ///////////
  // IMU Data
  sensor_msgs::msg::Imu imuMsg;
  imuMsg.header.stamp = tNow;
  // imuMsg.status - to do in the future.
  imuMsg.linear_acceleration.x = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.linear_acceleration.y = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.linear_acceleration.z = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.angular_velocity.x = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.angular_velocity.y = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.angular_velocity.z = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.orientation.x = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.orientation.y = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.orientation.z = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  imuMsg.orientation.w = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  pubImu_->publish(std::move(imuMsg));

  ////////////////////////
  // Depth and Temperature
  atl_msgs::msg::Depth depthMsg;
  depthMsg.header.stamp = tNow;
  // depthMsg.status = -1;
  depthMsg.depth = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  // depthMsg.temperature = 112.9;
  depthMsg.temperature = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  pubDepth_->publish(std::move(depthMsg));

  iter_++;
}


// ///////////////
// UDP Msg Forward
// ///////////////
void TeensyInterfaceComponent::subJoystickCb(sensor_msgs::msg::Joy::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);

  // if (msg->inputs.size() != prm_.n_servos) {
  //   RCLCPP_ERROR(
  //     get_logger(), "ServosInput message must have %lu inputs, received %lu.",
  //     prm_.n_servos, msg->inputs.size());
  //   throw std::runtime_error("ServosInput message has wrong number of inputs.");
  // }

  // forward message through UDP
  std::vector<uint8_t> u((nServos + 2) * 4); 
  std::size_t oft = 0;

  // Timestamp
  const uint64_t time_ns = now().nanoseconds();
  const uint32_t time_ms = static_cast<uint32_t>((time_ns - t0_) / 1000000U);
  memcpy(u.data() + oft, &time_ms, 4);
  oft += 4;
  // std::cout<<time_ms<<std::endl;

  // Servo Inputs
  for (std::size_t i = 0; i < nServos; i++) {
    // const float del = msg->inputs[i].delta;
    const float del = msg-> axes[i];
    memcpy(u.data() + oft, &del, 4);
    oft += 4;
  }
  // std::cout<< msg-> axes[0]<<std::endl;
  // std::cout<< msg-> axes[1]<<std::endl;


  // Sync byte
  const uint32_t sync = std::exchange(sync_, false);
  memcpy(u.data() + oft, &sync, 4);
  oft += 4;
  // std::cout<<sync<<std::endl;


  udp_->sendMsg(u);
}

}  // namespace atl

RCLCPP_COMPONENTS_REGISTER_NODE(atl::TeensyInterfaceComponent)
