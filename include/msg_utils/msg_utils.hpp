// Copyright 2023 ATL Robotics, USA

#ifndef TEENSY_INTERFACE__MSG_UTILS_HPP_
#define TEENSY_INTERFACE__MSG_UTILS_HPP_

#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #include <atl_msgs/msg/ctrl_state.hpp>
// #include <atl_msgs/msg/lla.hpp>
// #include <atl_msgs/msg/lla_pose.hpp>
// #include <atl_msgs/msg/engine_input.hpp>
// #include <atl_msgs/msg/engines_input.hpp>
// #include <atl_msgs/msg/boat.hpp>
// #include <atl_msgs/msg/dock.hpp>
// #include <atl_msgs/msg/poi.hpp>
// #include <atl_msgs/msg/sector.hpp>
// #include <atl_msgs/msg/marina.hpp>

#include <string_view>
#include <chrono>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "boat_info.hpp"
#include "atl_library/dynamics/types.hpp"
#include "geodetic.hpp"
#include "map/map.hpp"
#include "msg_types.hpp"
#include "utils.hpp"
#include "cyber_enum.hpp"

namespace atl::msgs
{

namespace detail
{
// Traits to check if it's a chrono_duration
template<typename T>
struct is_chrono_duration : std::false_type {};
template<typename R, typename P>
struct is_chrono_duration<std::chrono::duration<R, P>>: std::true_type {};
}

inline constexpr BOAT_MODE uint64ToBoatMode(const uint64_t u)
{
  if (u < static_cast<uint64_t>(BOAT_MODE::N_MODES)) {
    return static_cast<BOAT_MODE>(u);
  }
  return BOAT_MODE::UNKOWN;
}

inline constexpr uint64_t boatModeToUnit64(const BOAT_MODE m)
{
  return static_cast<uint64_t>(m);
}

inline constexpr CLUTCH int64ToClutch(const int64_t u)
{
  if (u > static_cast<int64_t>(CLUTCH::N_MIN) &&
    u < static_cast<int64_t>(CLUTCH::N_MAX))
  {
    return static_cast<CLUTCH>(u);
  }
  return CLUTCH::UNKOWN;
}

inline CLUTCH stringToClutch(const std::string_view s)
{
  if (strcmpi(s, "REVERSE")) {
    return CLUTCH::REVERSE;
  } else if (strcmpi(s, "NEUTRAL")) {
    return CLUTCH::NEUTRAL;
  } else if (strcmpi(s, "FORWARD")) {
    return CLUTCH::FORWARD;
  }
  return CLUTCH::UNKOWN;
}

inline constexpr int64_t clutchToInt64(const CLUTCH m)
{
  return static_cast<int64_t>(m);
}

inline constexpr DOCKING_RETURN_CODE int64ToDrc(const int64_t u)
{
  if (u > static_cast<int64_t>(DOCKING_RETURN_CODE::N_MIN) &&
    u < static_cast<int64_t>(DOCKING_RETURN_CODE::N_MAX))
  {
    return static_cast<DOCKING_RETURN_CODE>(u);
  }
  return DOCKING_RETURN_CODE::UNKOWN;
}

inline constexpr int64_t drcTonit64(const DOCKING_RETURN_CODE m)
{
  return static_cast<int64_t>(m);
}


struct EngineTypeEnum : CyberEnum<EngineTypeEnum>
{
  using CyberEnum<EngineTypeEnum>::CyberEnum;
  using CyberEnum<EngineTypeEnum>::operator=;

  static constexpr int unknown = 0;
  static constexpr int F350 = 1;
  static constexpr int F300 = 2;
  static constexpr int F200 = 3;
  static constexpr int XTO = 4;
  static constexpr int HARMO = 5;
  static constexpr int F300v2 = 6;

  static constexpr std::array values = {0, 1, 2, 3, 4, 5, 6};
  static constexpr std::array names = {"unknown", "F350", "F300", "F200", "XTO", "HARMO", "F300v2"};
};

/********************************************
    ROS types to Eigen/chrono types
*********************************************/

/**
 * Point/Vector3 message to Eigen Vector3d
 */
template<typename T>
inline std::enable_if_t<
  std::is_same_v<T, geometry_msgs::msg::Point>|| std::is_same_v<T, geometry_msgs::msg::Vector3>,
  Eigen::Vector3d
>
from_msg(const T & msg)
{
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

/**
 * Lla message to Eigen Vector3d
 */
inline Eigen::Vector3d from_msg(const atl_msgs::msg::Lla & msg)
{
  return Eigen::Vector3d(msg.lati, msg.longi, msg.alti);
}

/**
 * Twist message to Eigen Vector6d
 */
inline Eigen::Matrix<double, 6, 1> from_msg(const geometry_msgs::msg::Twist & msg)
{
  return (Eigen::Matrix<double, 6, 1>() << from_msg(msg.linear), from_msg(msg.angular)).finished();
}

/**
 * Quaternion message to Eigen quaternion
 */
inline Eigen::Quaterniond from_msg(const geometry_msgs::msg::Quaternion & msg)
{
  return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

/**
 * Pose message to Sophus SE3d
 */

inline Sophus::SE3d from_msg(const geometry_msgs::msg::Pose & msg)
{
  return Sophus::SE3d(from_msg(msg.orientation), from_msg(msg.position));
}

/**
 * Transform message to Sophus SE3d
 */
inline Sophus::SE3d from_msg(const geometry_msgs::msg::Transform & msg)
{
  return Sophus::SE3d(from_msg(msg.rotation), from_msg(msg.translation));
}

/**
 * LlaPose message to Sophus SE3d
 */
inline Sophus::SE3d from_msg(const atl_msgs::msg::LlaPose & msg)
{
  return Sophus::SE3d(from_msg(msg.orientation), from_msg(msg.position));
}

/**
 * ROS duration to std::chrono duration
 */
template<typename T>
std::enable_if_t<detail::is_chrono_duration<T>::value, T>
from_msg(const builtin_interfaces::msg::Time & stamp)
{
  return std::chrono::duration_cast<T>(
    std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)
  );
}


/**
 * atl_msgs::msg::EngineInput message to atl::dyn::EngineInput
 */
inline auto from_msg(const atl_msgs::msg::EngineInput & msg)
{
  return atl::dyn::EngineInput<double>{msg.thr, msg.delta, int64ToClutch(msg.clu)};
}


/**
 * atl_msgs::msg::EnginesInput message to atl::dyn::eng_input_t
 */
template<std::size_t _nEng>
atl::dyn::eng_input_t<_nEng, double>
from_msg(const atl_msgs::msg::EnginesInput & msg)
{
  if (msg.inputs.size() != _nEng) {
    throw std::invalid_argument("from_msg: wrong number of engines");
  }

  atl::dyn::eng_input_t<_nEng, double> ret;
  for (size_t i = 0; i < _nEng; ++i) {
    ret[i] = from_msg(msg.inputs[i]);
  }
  return ret;
}

/**
 * atl_msgs::msg::BoatInfo message to atl::BoatInfo
 */
inline BoatInfo::Geometry from_msg(const atl_msgs::msg::BoatInfoGeometry & msg)
{
  return BoatInfo::Geometry{msg.length, msg.width, msg.clearance, msg.draft, msg.weight,
    msg.n_engines};
}

/**
 * atl_msgs::msg::BoatInfo message to atl::BoatInfo
 */
inline BoatInfo from_msg(const atl_msgs::msg::BoatInfo & msg)
{
  return BoatInfo{msg.docu.name, msg.docu.hin, msg.docu.category, from_msg(msg.geo), msg.date};
}

/**
 * atl_msgs::msg::Dock message to atl::map::Dock
 */
inline map::Dock from_msg(const atl_msgs::msg::Dock & msg)
{
  map::Dock dock;

  dock.name = msg.name;
  dock.sector = msg.sector;
  dock.date = msg.date;
  dock.pose = from_msg(msg.pose);
  dock.approach_pose.active = msg.approach_pose.active;
  dock.approach_pose.pose = from_msg(msg.approach_pose.pose);
  dock.length = msg.length;
  dock.width = msg.width;
  dock.occupied = msg.occupied;
  dock.topology.x_plus = msg.topology.x_plus;
  dock.topology.x_minus = msg.topology.x_minus;
  dock.topology.y_plus = msg.topology.y_plus;
  dock.topology.y_minus = msg.topology.y_minus;
  dock.pozyx.active = msg.pozyx.active;
  dock.pozyx.name = msg.pozyx.name;
  dock.pozyx.date = msg.pozyx.date;
  dock.pozyx.dock_offset = from_msg(msg.pozyx.dock_offset);
  dock.pumpout = msg.pumpout;
  dock.fuels.reserve(msg.fuels.size());
  for (const auto & fuel : msg.fuels) {
    dock.fuels.push_back({fuel.type, fuel.price});
  }
  return dock;
}

/**
 * atl_msgs::msg::Boat message to atl::map::Boat
 */
inline map::Boat from_msg(const atl_msgs::msg::Boat & msg)
{
  map::Boat boat;

  boat.name = msg.name;
  boat.sector = msg.sector;
  boat.date = msg.date;
  boat.pose = from_msg(msg.pose);
  boat.vel = from_msg(msg.vel);
  boat.info = from_msg(msg.info);

  return boat;
}

/**
 * atl_msgs::msg::POI message to atl::map::POI
 */
inline map::POI from_msg(const atl_msgs::msg::POI & msg)
{
  map::POI poi;

  poi.name = msg.name;
  poi.sector = msg.sector;
  poi.date = msg.date;
  poi.type = msg.type;
  poi.pose = from_msg(msg.pose);

  return poi;
}

inline map::Sector::Route from_msg(const atl_msgs::msg::SectorRoute & msg)
{
  map::Sector::Route route;
  route.from = msg.src;
  route.to = msg.dest;
  route.two_way = msg.two_way;
  route.boat_limits = from_msg(msg.boat_limits);
  return route;
}
inline map::Sector::Junction from_msg(const atl_msgs::msg::SectorJunction & msg)
{
  map::Sector::Junction junction;
  junction.boundary.first = msg.boundary[0];
  junction.boundary.second = msg.boundary[1];
  junction.sector_name = msg.sector_name;
  junction.sector_waypoint = msg.sector_waypoint;
  return junction;
}

inline map::Sector from_msg(const atl_msgs::msg::Sector & msg)
{
  map::Sector sector;
  sector.name = msg.name;
  sector.date = msg.date;
  sector.type = msg.type;
  sector.allowed_boat_categories = msg.allowed_boat_categories;

  sector.boundaries.reserve(msg.boundaries.size());
  for (const auto & boundary : msg.boundaries) {
    std::vector<Eigen::Vector3d> vtx;
    vtx.reserve(boundary.vertices.size());
    for (const auto & v : boundary.vertices) {
      vtx.push_back(from_msg(v));
    }
    sector.boundaries.push_back(std::move(vtx));
  }

  sector.waypoints.reserve(msg.waypoints.size());
  for (const auto & w : msg.waypoints) {
    sector.waypoints.push_back(from_msg(w));
  }

  sector.routes.reserve(msg.routes.size());
  for (const auto & r : msg.routes) {
    sector.routes.push_back(from_msg(r));
  }

  sector.junctions.reserve(msg.junctions.size());
  for (const auto & j : msg.junctions) {
    sector.junctions.push_back(from_msg(j));
  }

  return sector;
}

inline map::Marina::Services from_msg(const atl_msgs::msg::MarinaServices & msg)
{
  map::Marina::Services services;
  services.pumpout_station = msg.pumpout_station;
  services.fuel_station = msg.fuel_station;
  return services;
}

inline map::Marina from_msg(const atl_msgs::msg::Marina & msg)
{
  map::Marina marina;
  marina = from_msg(msg.sector);
  marina.services = from_msg(msg.services);
  marina.center = from_msg(msg.center);
  return marina;
}


/********************************************
    Eigen/chrono types to ROS types
*********************************************/

/**
 * Eigen Vector3 to Point/Vector3/Lla message
 */
template<typename T = geometry_msgs::msg::Point, typename Derived>
std::enable_if_t<
  Derived::IsVectorAtCompileTime && Derived::SizeAtCompileTime == 3 &&
  (std::is_same_v<T, geometry_msgs::msg::Point>||
  std::is_same_v<T, geometry_msgs::msg::Vector3>||
  std::is_same_v<T, atl_msgs::msg::Lla>),
  T
>
to_msg(const Eigen::MatrixBase<Derived> & p)
{
  T ret;
  if constexpr (std::is_same_v<T, atl_msgs::msg::Lla>) {
    ret.lati = p.x();
    ret.longi = p.y();
    ret.alti = p.z();
  } else {
    ret.x = p.x();
    ret.y = p.y();
    ret.z = p.z();
  }
  return ret;
}

/**
 * Eigen Vector6 to Twist message
 */
template<typename T = geometry_msgs::msg::Twist, typename Derived>
std::enable_if_t<
  Derived::IsVectorAtCompileTime && Derived::SizeAtCompileTime == 6 &&
  std::is_same_v<T, geometry_msgs::msg::Twist>,
  T
>
to_msg(const Eigen::MatrixBase<Derived> & p)
{
  geometry_msgs::msg::Twist ret;
  ret.linear = to_msg<geometry_msgs::msg::Vector3>(p.template head<3>());
  ret.angular = to_msg<geometry_msgs::msg::Vector3>(p.template tail<3>());
  return ret;
}

/**
 * Eigen Quaternion to Quaternion msg
 */
template<typename T = geometry_msgs::msg::Quaternion, typename Scalar>
std::enable_if_t<std::is_same_v<T, geometry_msgs::msg::Quaternion>, T>
to_msg(const Eigen::Quaternion<Scalar> & q)
{
  T ret;
  ret.w = q.w();
  ret.x = q.x();
  ret.y = q.y();
  ret.z = q.z();
  return ret;
}

/**
 * Sophus SE3 to Pose/Transform message
 */
template<typename T = geometry_msgs::msg::Pose, typename Scalar>
std::enable_if_t<
  std::is_same_v<T, geometry_msgs::msg::Pose>||
  std::is_same_v<T, geometry_msgs::msg::Transform>||
  std::is_same_v<T, atl_msgs::msg::LlaPose>,
  T
>
to_msg(const Sophus::SE3<Scalar> & pose)
{
  T ret;
  if constexpr (std::is_same_v<T, geometry_msgs::msg::Pose>) {
    ret.position = to_msg<geometry_msgs::msg::Point>(pose.translation());
    ret.orientation = to_msg(pose.unit_quaternion());
  }
  if constexpr (std::is_same_v<T, geometry_msgs::msg::Transform>)
  {
    ret.translation = to_msg<geometry_msgs::msg::Vector3>(pose.translation());
    ret.rotation = to_msg(pose.unit_quaternion());
  }
  if constexpr (std::is_same_v<T, atl_msgs::msg::LlaPose>)
  {
    ret.position = to_msg<atl_msgs::msg::Lla>(pose.translation());
    ret.orientation = to_msg(pose.unit_quaternion());
  }
  return ret;
}

/**
 * std::chrono duration to ROS duration
 */
template<typename T = builtin_interfaces::msg::Time, typename S>
std::enable_if_t<detail::is_chrono_duration<S>::value, T>
to_msg(const S & t)
{
  T ret;
  auto t_sec = std::chrono::duration_cast<std::chrono::seconds>(t);
  ret.sec = t_sec.count();
  ret.nanosec = std::chrono::nanoseconds(
    std::chrono::duration_cast<std::chrono::nanoseconds>(t) - t_sec).count();
  return ret;
}

/**
 * atl::dyn::EngineInput to atl_msgs::msg::EngineInput message
 */
template<typename T>
atl_msgs::msg::EngineInput to_msg(const dyn::EngineInput<T> & in)
{
  atl_msgs::msg::EngineInput ret;
  ret.thr = in.thr;
  ret.delta = in.delta;
  ret.clu = clutchToInt64(in.clu);
  return ret;
}

/**
 * atl::dyn::eng_input_t to atl_msgs::msg::EnginesInput message
 */
template<std::size_t _nEng, typename _scalar = double>
atl_msgs::msg::EnginesInput to_msg(const atl::dyn::eng_input_t<_nEng, _scalar> & in)
{
  atl_msgs::msg::EnginesInput ret;
  for (size_t i = 0; i < _nEng; ++i) {
    ret.inputs.push_back(to_msg(in[i]));
  }
  // Not sure how to handle the run mode - included in message but not in type
  // Implicitly 0 = IDLE
  return ret;
}


inline atl_msgs::msg::BoatInfoGeometry to_msg(const BoatInfo::Geometry & in)
{
  atl_msgs::msg::BoatInfoGeometry msg;

  msg.length = in.length;
  msg.width = in.width;
  msg.clearance = in.clearance;
  msg.draft = in.draft;
  msg.weight = in.weight;
  msg.n_engines = in.n_engines;

  return msg;
}
/**
 * atl::BoatInfo to atl_msgs::msg::BoatInfo message
 */
inline atl_msgs::msg::BoatInfo to_msg(const BoatInfo & in)
{
  atl_msgs::msg::BoatInfo msg;

  msg.docu.name = in.docu.name;
  msg.docu.hin = in.docu.hin;
  msg.docu.category = in.docu.category;
  msg.geo = to_msg(in.geo);
  msg.date = in.date;

  return msg;
}

/**
 * atl::map::Dock to atl_msgs::msg::Dock message
 */
inline atl_msgs::msg::Dock to_msg(const map::Dock & in)
{
  atl_msgs::msg::Dock msg;

  msg.name = in.name;
  msg.sector = in.sector;
  msg.date = in.date;
  msg.pose = to_msg<atl_msgs::msg::LlaPose>(in.pose.to_sophus());
  msg.approach_pose.active = in.approach_pose.active;
  msg.approach_pose.pose = to_msg<atl_msgs::msg::LlaPose>(in.approach_pose.pose.to_sophus());
  msg.length = in.length;
  msg.width = in.width;
  msg.occupied = in.occupied;
  msg.topology.x_plus = in.topology.x_plus;
  msg.topology.x_minus = in.topology.x_minus;
  msg.topology.y_plus = in.topology.y_plus;
  msg.topology.y_minus = in.topology.y_minus;
  msg.pozyx.active = in.pozyx.active;
  msg.pozyx.name = in.pozyx.name;
  msg.pozyx.date = in.pozyx.date;
  msg.pozyx.dock_offset = to_msg<geometry_msgs::msg::Pose>(in.pozyx.dock_offset.to_sophus());
  msg.pumpout = in.pumpout;
  msg.fuels.reserve(in.fuels.size());
  for (const auto & fuel : in.fuels) {
    atl_msgs::msg::DockFuelInfo info;
    info.type = fuel.type;
    info.price = fuel.price;
    msg.fuels.push_back(std::move(info));
  }
  return msg;
}

/**
 * atl::map::Boat to atl_msgs::msg::Boat message
 */
inline atl_msgs::msg::Boat to_msg(const map::Boat & boat)
{
  atl_msgs::msg::Boat msg;

  msg.name = boat.name;
  msg.sector = boat.sector;
  msg.date = boat.date;
  msg.pose = to_msg<atl_msgs::msg::LlaPose>(boat.pose.to_sophus());
  msg.vel = to_msg(boat.vel.to_sophus());
  msg.info = to_msg(boat.info);

  return msg;
}

/**
 * atl::map::POI to atl_msgs::msg::POI message
 */
inline atl_msgs::msg::POI to_msg(const map::POI & poi)
{
  atl_msgs::msg::POI msg;

  msg.name = poi.name;
  msg.sector = poi.sector;
  msg.date = poi.date;
  msg.type = poi.type;
  msg.pose = to_msg<atl_msgs::msg::LlaPose>(poi.pose.to_sophus());

  return msg;
}

inline atl_msgs::msg::SectorRoute to_msg(const map::Sector::Route & route)
{
  atl_msgs::msg::SectorRoute msg;

  msg.src = route.from;
  msg.dest = route.to;
  msg.two_way = route.two_way;
  msg.boat_limits = to_msg(route.boat_limits);

  return msg;
}

inline atl_msgs::msg::SectorJunction to_msg(const map::Sector::Junction & junction)
{
  atl_msgs::msg::SectorJunction msg;

  msg.boundary[0] = junction.boundary.first;
  msg.boundary[1] = junction.boundary.second;
  msg.sector_name = junction.sector_name;
  msg.sector_waypoint = junction.sector_waypoint;

  return msg;
}

inline atl_msgs::msg::Sector to_msg(const map::Sector & sector)
{
  atl_msgs::msg::Sector msg;
  msg.name = sector.name;
  msg.date = sector.date;
  msg.type = sector.type;
  msg.allowed_boat_categories = sector.allowed_boat_categories;

  msg.boundaries.reserve(sector.boundaries.size());
  for (const auto & boundary : sector.boundaries) {
    atl_msgs::msg::SectorBoundary vtx;
    vtx.vertices.reserve(boundary.size());
    vtx.active.reserve(boundary.size());
    for (const auto & v : boundary) {
      vtx.vertices.push_back(to_msg<atl_msgs::msg::Lla>(v));
      vtx.active.push_back(true);
    }
    msg.boundaries.push_back(std::move(vtx));
  }

  msg.waypoints.reserve(sector.waypoints.size());
  for (const auto & w : sector.waypoints) {
    msg.waypoints.push_back(to_msg<atl_msgs::msg::Lla>(w));
  }

  msg.routes.reserve(sector.routes.size());
  for (const auto & r : sector.routes) {
    msg.routes.push_back(to_msg(r));
  }

  msg.junctions.reserve(sector.junctions.size());
  for (const auto & j : sector.junctions) {
    msg.junctions.push_back(to_msg(j));
  }

  return msg;
}

inline atl_msgs::msg::MarinaServices to_msg(const map::Marina::Services & services)
{
  atl_msgs::msg::MarinaServices msg;

  msg.pumpout_station = services.pumpout_station;
  msg.fuel_station = services.fuel_station;

  return msg;
}

inline atl_msgs::msg::Marina to_msg(const map::Marina & marina)
{
  atl_msgs::msg::Marina msg;
  msg.sector = to_msg(static_cast<map::Sector>(marina));
  msg.services = to_msg(marina.services);
  msg.center = to_msg<atl_msgs::msg::Lla>(marina.center.as_sophus());
  return msg;
}

inline geometry_msgs::msg::Vector3 point_vec3(const geometry_msgs::msg::Point & p)
{
  geometry_msgs::msg::Vector3 out;
  out.x = p.x;
  out.y = p.y;
  out.z = p.z;
  return out;
}

inline geometry_msgs::msg::Point point_vec3(const geometry_msgs::msg::Vector3 & v)
{
  geometry_msgs::msg::Point out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

template<typename _ref = ::atl::geo::WGS84, typename ... Args>
auto imu2nwu(Args && ... args)
{
  return msgs::to_msg(::atl::geo::imu2nwu<_ref>(msgs::from_msg(std::forward<Args>(args))...));
}

template<typename _ref = ::atl::geo::WGS84, typename ... Args>
auto nwu2imu(Args && ... args)
{
  const auto lla = ::atl::geo::nwu2imu<_ref>(msgs::from_msg(std::forward<Args>(args))...);
  if constexpr (std::is_same_v<std::decay_t<decltype(lla)>, Sophus::SE3d>) {
    return msgs::to_msg<atl_msgs::msg::LlaPose>(lla);
  } else {
    return msgs::to_msg(lla);
  }
}

template<typename _ref = ::atl::geo::WGS84, typename ... Args>
auto lla2nwu(Args && ... args)
{
  return msgs::to_msg(::atl::geo::lla2nwu<_ref>(msgs::from_msg(std::forward<Args>(args))...));
}


template<typename _ref = ::atl::geo::WGS84, typename ... Args>
auto nwu2lla(Args && ... args)
{
  const auto lla = ::atl::geo::nwu2lla<_ref>(msgs::from_msg(std::forward<Args>(args))...);
  if constexpr (std::is_same_v<std::decay_t<decltype(lla)>, Eigen::Vector3d>) {
    return msgs::to_msg<atl_msgs::msg::Lla>(lla);
  } else {
    return msgs::to_msg<atl_msgs::msg::LlaPose>(lla);
  }
}

}  // namespace atl::msgs

#endif  // atl_LIBRARY__MSG_UTILS_HPP_
