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

constexpr static std::size_t nServos = 5;

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

  pubLeak_ = create_publisher<atl_msgs::msg::Leak>(
    "leak", rclcpp::SystemDefaultsQoS());

  pubServos_ = create_publisher<atl_msgs::msg::ServosFeedback>(
    "servo_feedback", rclcpp::SystemDefaultsQoS());

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

  tf_broadBoat_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_broadBody_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_broadActuator1_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_broadActuator2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "Teensy Interface Node started");
}


//////////////////
// UDP Msg Receive
//////////////////
void TeensyInterfaceComponent::udpCb(const UDPServer::UDPMsg & msg)
{
  // Lin_acc(3) + Ang_vel(3) + Quat(4) + depth(1) + temp(1) + leak(1) + servo(5)
  constexpr std::size_t msgLen = (3+3+4+1+1+1+5) * 4;

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

  //////////////
  // Leak sensor
  atl_msgs::msg::Leak leakMsg;
  leakMsg.header.stamp = tNow;
  leakMsg.leak = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  pubLeak_->publish(std::move(leakMsg));


  /////////////////
  // Servo Feedback
  
  atl_msgs::msg::ServoFeedback servoMsg1;
  servoMsg1.header.stamp = tNow;
  servoMsg1.delta = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;




  atl_msgs::msg::ServoFeedback servoMsg2;
  servoMsg2.header.stamp = tNow;
  servoMsg2.delta = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  atl_msgs::msg::ServoFeedback servoMsg3;
  servoMsg3.header.stamp = tNow;
  servoMsg3.delta = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  atl_msgs::msg::ServoFeedback servoMsg4;
  servoMsg4.header.stamp = tNow;
  servoMsg4.delta = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  atl_msgs::msg::ServoFeedback servoMsg5;
  servoMsg5.header.stamp = tNow;
  servoMsg5.delta = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;



  atl_msgs::msg::ServosFeedback servosMsg;
  servosMsg.feedback.resize(5);

  servosMsg.feedback[0].delta = servoMsg1.delta;
  servosMsg.feedback[1].delta = servoMsg2.delta;
  servosMsg.feedback[2].delta = servoMsg3.delta;
  servosMsg.feedback[3].delta = servoMsg4.delta;
  servosMsg.feedback[4].delta = servoMsg5.delta;
  
  pubServos_->publish(std::move(servosMsg));


  /////////////
  // Transforms
  geometry_msgs::msg::TransformStamped t_boat;
  geometry_msgs::msg::TransformStamped t_body;
  geometry_msgs::msg::TransformStamped t_act1;
  geometry_msgs::msg::TransformStamped t_act2;

  // --------- boat
  t_boat.header.stamp = tNow;
  t_boat.header.frame_id = "world";
  t_boat.child_frame_id = "boat";
  // Boat Translation
  t_boat.transform.translation.x = 0.0;
  t_boat.transform.translation.y = 0.0;
  t_boat.transform.translation.z = 0.25;
  // Left actuator Rotation
  tf2::Quaternion q0;
  q0.setRPY(0, 0, 0);
  t_boat.transform.rotation.x = q0.x();
  t_boat.transform.rotation.y = q0.y();
  t_boat.transform.rotation.z = q0.z();
  t_boat.transform.rotation.w = q0.w();
  // Send the transformation
  tf_broadBoat_->sendTransform(t_boat);

  // --------- Paravane
  t_body.header.stamp = tNow;
  t_body.header.frame_id = "boat";
  t_body.child_frame_id = "paravane";
  // Paravane Translation
  t_body.transform.translation.x = -5.0;
  t_body.transform.translation.y = 0.0;
  t_body.transform.translation.z = -depthMsg.depth;
  // Paravane Rotation
  t_body.transform.rotation.x = imuMsg.orientation.x;
  t_body.transform.rotation.y = imuMsg.orientation.y;
  t_body.transform.rotation.z = imuMsg.orientation.z;
  t_body.transform.rotation.w = imuMsg.orientation.w;
  // Send the transformation
  tf_broadBody_->sendTransform(t_body);

// --------- Main Wing actuator
  t_act1.header.stamp = tNow;
  t_act1.header.frame_id = "paravane";
  t_act1.child_frame_id = "main_wing";
  // Main Wing actuator Translation
  t_act1.transform.translation.x = 0.1;
  t_act1.transform.translation.y = 0.0;
  t_act1.transform.translation.z = 0.0;
  // Main Wing actuator Rotation
  tf2::Quaternion q1;
  q1.setRPY(0, del1_, 0);
  t_act1.transform.rotation.x = q1.x();
  t_act1.transform.rotation.y = q1.y();
  t_act1.transform.rotation.z = q1.z();
  t_act1.transform.rotation.w = q1.w();
  // Send the transformation
  tf_broadBody_->sendTransform(t_act1);

  // --------- Top Fin actuator
  t_act1.header.stamp = tNow;
  t_act1.header.frame_id = "paravane";
  t_act1.child_frame_id = "actuator1";
  // Actuator Translation
  t_act1.transform.translation.x = -0.2;
  t_act1.transform.translation.y = 0.0;
  t_act1.transform.translation.z = 0.05;
  // Actuator Rotation
  tf2::Quaternion q1;
  q1.setRPY(0, 0, del2_);
  t_act1.transform.rotation.x = q1.x();
  t_act1.transform.rotation.y = q1.y();
  t_act1.transform.rotation.z = q1.z();
  t_act1.transform.rotation.w = q1.w();
  // Send the transformation
  tf_broadBody_->sendTransform(t_act1);



  // // --------- Left actuator
  // t_act1.header.stamp = tNow;
  // t_act1.header.frame_id = "paravane";
  // t_act1.child_frame_id = "actuator1";
  // // Left actuator Translation
  // t_act1.transform.translation.x = -0.2075;
  // t_act1.transform.translation.y = 0.0085;
  // t_act1.transform.translation.z = 0.0;
  // // Left actuator Rotation
  // tf2::Quaternion q1;
  // q1.setRPY(0, del1_, 0);
  // t_act1.transform.rotation.x = q1.x();
  // t_act1.transform.rotation.y = q1.y();
  // t_act1.transform.rotation.z = q1.z();
  // t_act1.transform.rotation.w = q1.w();
  // // Send the transformation
  // tf_broadBody_->sendTransform(t_act1);

  // // --------- Right Actuator
  // t_act2.header.stamp = tNow;
  // t_act2.header.frame_id = "paravane";
  // t_act2.child_frame_id = "actuator2";
  // // Right Actuator Translation
  // t_act2.transform.translation.x = -0.2075;
  // t_act2.transform.translation.y = -0.0085;
  // t_act2.transform.translation.z = 0.0;
  // // Right Actuator Rotation
  // tf2::Quaternion q2;
  // q2.setRPY(0, del2_, 0);
  // t_act2.transform.rotation.x = q2.x();
  // t_act2.transform.rotation.y = q2.y();
  // t_act2.transform.rotation.z = q2.z();
  // t_act2.transform.rotation.w = q2.w();
  // // Send the transformation
  // tf_broadBody_->sendTransform(t_act2);

  iter_++;
}


// ///////////////
// UDP Msg Forward
// ///////////////
void TeensyInterfaceComponent::subJoystickCb(sensor_msgs::msg::Joy::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);

  // forward message through UDP
  std::vector<uint8_t> u((nServos + 2) * 4); 
  std::size_t oft = 0;

  // Timestamp
  const uint64_t time_ns = now().nanoseconds();
  const uint32_t time_ms = static_cast<uint32_t>((time_ns - t0_) / 1000000U);
  memcpy(u.data() + oft, &time_ms, 4);
  oft += 4;

  // Servo Inputs
  del1_ = msg-> axes[1];
  memcpy(u.data() + oft, &del1_, 4);
  oft += 4;

  del2_ = msg-> axes[4];
  memcpy(u.data() + oft, &del2_, 4);
  oft += 4;

  del3_ = 0;
  memcpy(u.data() + oft, &del3_, 4);
  oft += 4;

  del4_ = 0;
  memcpy(u.data() + oft, &del4_, 4);
  oft += 4;

  del5_ = 0;
  memcpy(u.data() + oft, &del5_, 4);
  oft += 4;

  // Sync byte
  const uint32_t sync = std::exchange(sync_, false);
  memcpy(u.data() + oft, &sync, 4);
  oft += 4;


  udp_->sendMsg(u);
}

}  // namespace atl

RCLCPP_COMPONENTS_REGISTER_NODE(atl::TeensyInterfaceComponent)
