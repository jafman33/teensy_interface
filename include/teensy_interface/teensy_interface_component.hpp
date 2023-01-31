// Copyright 2023 ATL Robotics, USA

#ifndef TEENSY_INTERFACE__TEENSY_INTERFACE_COMPONENT_HPP_
#define TEENSY_INTERFACE__TEENSY_INTERFACE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <boost/fusion/adapted/struct.hpp>

#include <atl_library/clock_traits_ros.hpp>
#include <atl_library/integrator.hpp>
#include <atl_library/msg_utils.hpp>

#include <atl_msgs/msg/servo_input.hpp>
#include <atl_msgs/msg/wing_state.hpp>
// #include <atl_msgs/msg/bcu_status.hpp>
// #include <atl_msgs/msg/lights.hpp>
// #include <atl_msgs/msg/buzzers.hpp>
// #include <atl_msgs/msg/trims_input.hpp>
// #include <atl_msgs/msg/engines_power_input.hpp>
// #include <atl_msgs/msg/fuel_tanks.hpp>
// #include <atl_msgs/msg/ctrl_status.hpp>
// #include <atl_msgs/msg/cortex_status.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include "udp_server/udp_server.hpp"

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
  std::string mab_ip{"192.168.1.3"};
  uint16_t send_port{1560};
  uint16_t receive_port{1561};
  uint32_t receive_buffer_size{1024};

  void check_correctness() const
  {
    if (mab_ip.empty()) {
      throw std::invalid_argument("mab_ip parameter must not be empty.");
    }
    if (receive_buffer_size == 0) {
      throw std::invalid_argument("receive_buffer_size parameter must be > 0.");
    }
  }
};

struct TeensyInterfaceTrimParams
{
  double sensor_threshold{0.43};
  double trim_to_tilt_threshold{0.356};
  double v_tilt_up{0.1174};
  double v_tilt_down{-0.1374};
  double v_trim_up{0.0376};
  double v_trim_down{-0.0303};
  double sat_min{0.};
  double sat_max{0.9};

  void check_correctness() const
  {
    if (sensor_threshold <= 0.) {
      throw std::invalid_argument("parameter 'sensor_threshold' must be stricly greater than 0");
    }
    if (trim_to_tilt_threshold <= 0.) {
      throw std::invalid_argument(
              "parameter 'trim_to_tilt_threshold' must be stricly greater than 0");
    }
    if (v_tilt_up <= 0.) {
      throw std::invalid_argument("parameter 'v_tilt_up' must be stricly greater than 0");
    }
    if (v_tilt_down >= 0.) {
      throw std::invalid_argument("parameter 'v_tilt_down' must be stricly less than 0");
    }
    if (v_trim_up <= 0.) {
      throw std::invalid_argument("parameter 'v_trim_up' must be stricly greater than 0");
    }
    if (v_trim_down >= 0.) {
      throw std::invalid_argument("parameter 'v_trim_down' must be stricly less than 0");
    }
    if (sat_min >= sat_max) {
      throw std::invalid_argument("parameter 'sat_min' must be stricly less than 'sat_max'");
    }
  }
};

struct TeensyInterfaceParams
{
  std::size_t n_engines{};

  TeensyUdpParams udp;
  TeensyInterfaceTrimParams trim;

  void check_correctness() const
  {
    if (n_engines < 1) {
      throw std::invalid_argument("n_engines parameter must be > 0.");
    }

    udp.check_correctness();
    trim.check_correctness();
  }
};

// Component declaration
class TeensyInterfaceComponent : public rclcpp::Node
{
public:
  explicit TeensyInterfaceComponent(const rclcpp::NodeOptions &);

private:
  rclcpp::Subscription<atl_msgs::msg::EnginesInput>::SharedPtr subInput_;
  rclcpp::Subscription<atl_msgs::msg::Buzzers>::SharedPtr subBuzz_;
  rclcpp::Subscription<atl_msgs::msg::Lights>::SharedPtr subPanelLights_;
  rclcpp::Subscription<atl_msgs::msg::Lights>::SharedPtr subHMLights_;
  rclcpp::Subscription<atl_msgs::msg::TrimsInput>::SharedPtr subEngineTrim_;
  rclcpp::Subscription<atl_msgs::msg::TrimsInput>::SharedPtr subTrimTabs_;
  rclcpp::Subscription<atl_msgs::msg::EnginesPowerInput>::SharedPtr subEnginePower_;
  rclcpp::Subscription<atl_msgs::msg::CtrlStatus>::SharedPtr subCtrlStatus_;
  rclcpp::Subscription<atl_msgs::msg::CortexStatus>::SharedPtr subCortexStatus_;

  rclcpp::Publisher<atl_msgs::msg::BcuStatus>::SharedPtr pubStatus_;
  rclcpp::Publisher<atl_msgs::msg::EnginesState>::SharedPtr pubEngine_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pubHM_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pubPanel_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pubSteer_;
  rclcpp::Publisher<atl_msgs::msg::FuelTanks>::SharedPtr pubFuel_;

  void subInputCb(atl_msgs::msg::EnginesInput::SharedPtr && msg);
  void subBuzzCb(atl_msgs::msg::Buzzers::SharedPtr && msg);
  void subPanelLightsCb(atl_msgs::msg::Lights::SharedPtr && msg);
  void subHMLightsCb(atl_msgs::msg::Lights::SharedPtr && msg);
  void subEngineTrimCb(atl_msgs::msg::TrimsInput::SharedPtr && msg);
  void subEnginePowerCb(atl_msgs::msg::EnginesPowerInput::SharedPtr && msg);
  void subTrimTabsCb(atl_msgs::msg::TrimsInput::SharedPtr && msg);
  void subCtrlStatusCb(atl_msgs::msg::CtrlStatus::SharedPtr && msg);
  void subCortexStatusCB(atl_msgs::msg::CortexStatus::SharedPtr && msg);

  void udpCb(const UDPServer::UDPMsg & msg);

  std::unique_ptr<UDPServer> udp_;
  uint64_t t0_;
  std::size_t iter_ = 0;
  bool sync_ = true;

  TeensyInterfaceParams prm_;

  uint32_t task_ = 0;
  uint32_t ctrlMode_ = 0;

  std::mutex msgMtx_;
  std::vector<std::optional<std::pair<uint8_t, int>>> buzzMsg_;
  atl_msgs::msg::Lights::SharedPtr panelLightsMsg_;
  atl_msgs::msg::Lights::SharedPtr HMLightsMsg_;
  atl_msgs::msg::TrimsInput::SharedPtr engineTrimMsg_;
  atl_msgs::msg::TrimsInput::SharedPtr trimTabsMsg_;

  
  std::vector<int32_t> enginePowerData_;

  // Trim related data members
  std::vector<int> userTrimInput_;
  std::vector<int> desTrimInput_;
  std::vector<Integrator<rclcpp::Clock>> trimIntegrators_;
  bool trimIntegratorsInit_ = false;
};

}  // namespace atl

// cppcheck-suppress unknownMacro
BOOST_FUSION_ADAPT_STRUCT(
  atl::TeensyUdpParams,
  mab_ip,
  send_port,
  receive_port,
  receive_buffer_size
)

// cppcheck-suppress unknownMacro
BOOST_FUSION_ADAPT_STRUCT(
  atl::TeensyInterfaceTrimParams,
  sensor_threshold,
  trim_to_tilt_threshold,
  v_tilt_up,
  v_tilt_down,
  v_trim_up,
  v_trim_down,
  sat_max,
  sat_min
)

// cppcheck-suppress unknownMacro
BOOST_FUSION_ADAPT_STRUCT(
  atl::TeensyInterfaceParams,
  n_engines,
  udp,
  trim
)

#endif  // MAB_INTERFACE__MAB_INTERFACE_COMPONENT_HPP_
