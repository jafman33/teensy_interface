// Copyright 2023 ATL Robotics, USA

#include "teensy_interface/teensy_interface_component.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec{};
  auto options = rclcpp::NodeOptions{};
  auto teensy_comp = std::make_shared<bps::TeensyInterfaceComponent>(options);

  exec.add_node(teensy_comp);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
