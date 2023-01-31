// Copyright 2023 ATL Robotics, USA

/*
Convencience methods for dealing with parameters.

Parameters often exist in three places: as ROS parameters, as members of
a struct, and in .yaml config files. These methods use boost::fusion template
programming to enable seamless typed conversion between those different representations.

Example usage:

1. Parameter objects must be adapted or defined as boost fusion structs

#include <boost/fusion/adapted/struct/define_struct.hpp>
#include <boost/fusion/include/define_struct.hpp>

struct Parameters
{
  int value1;
  double value2;
  int value3{};  // invisible in adaptation, should be default-initialized
};

BOOST_FUSION_ADAPT_STRUCT(
  Parameters
  (int, value1)
  (double, value2)
)

Alternatively, the struct can be defined directly with the BOOST_FUSION_DEFINE_STRUCT macro,
but this does not allow partial member visibility.


2. To and from YAML

  a) Read values into the struct from a yaml object

  YAML::Node yaml = YAML::Load("{value1: 1, value2: 1.5}");
  auto params = yaml.as<Parameters>();

  b) Convert a struct to a yaml object

  Parameters params{5, 5.6};
  auto yaml = YAML::Node(params);


3. To and from ROS

  a) Declare parameters in a ROS node. Values in the struct are used as default values

  Parameters params{5, 5.6};
  atl::declareParamsInNode<Parameters>(node, params);

  b) Read parameters values from a ROS node into a struct

  auto config = atl::readParamsFromNode<Parameters>(node);


4. Write parameter values in yaml format with spdlog

  Parameters params{5, 5.6};
  spdlog::info(The parameters are\n:{}, params);


Changelog
=========

 * 2020-05: Petter Nilsson created first version

*/

#ifndef TEENSY_INTERFACE__PARAMETER_HPP_
#define TEENSY_INTERFACE__PARAMETER_HPP_

#include <boost/mpl/range_c.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/fusion/adapted/struct.hpp>
#include <boost/fusion/adapted/struct/detail/extension.hpp>
#include <boost/fusion/include/at.hpp>
#include <boost/fusion/include/value_at.hpp>
#include <boost/fusion/support/is_sequence.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/type_index.hpp>

#include <fmt/format.h>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter_map.hpp>

#include <yaml-cpp/yaml.h>

#include <experimental/type_traits>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "utils.hpp"
#include "yaml.hpp"

using boost::fusion::extension::struct_member_name;
using boost::fusion::traits::is_sequence;

using std::experimental::is_detected_v;


namespace atl
{

namespace param_details
{

// Discrete
template<typename T>
using has_check_correctness_t = decltype(std::declval<T &>().check_correctness());

// Check if value fits in output type
template<typename T, typename T2>
void check_representable(const T2 & v, const std::string_view prmName)
{
  if constexpr (std::is_unsigned_v<T>) {
    if (v < 0) {
      throw std::runtime_error(
              fmt::format(
                "Parameter {} is declared unsigned but value is negative : {}",
                prmName, v));
    }
    if constexpr (!std::is_same_v<T, uint64_t>) {
      const auto m = static_cast<int64_t>(std::numeric_limits<T>::max());
      if (v > m) {
        throw std::runtime_error(
                fmt::format(
                  "Parameter {} value is greated than the maximum "
                  "representable number by the parameter type : {} > {}",
                  prmName, v, m));
      }
    }
  } else {
    const auto mi = static_cast<int64_t>(std::numeric_limits<T>::min());
    const auto ma = static_cast<int64_t>(std::numeric_limits<T>::max());
    if (v > ma) {
      throw std::runtime_error(
              fmt::format(
                "Parameter {} value is greated than the maximum "
                "representable number by the parameter type : {} > {}",
                prmName, v, ma));
    }

    if (v < mi) {
      throw std::runtime_error(
              fmt::format(
                "Parameter {} value is less than the minimum "
                "representable number by the parameter type : {} < {}",
                prmName, v, mi));
    }
  }
}

template<typename T>
void check_fit_int64([[maybe_unused]] const T & v, [[maybe_unused]] const std::string_view prmName)
{
  if constexpr (is_scoped_enum_v<T>) {
    if constexpr (std::is_same_v<std::underlying_type_t<T>, uint64_t>) {
      const auto ma = static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
      if (static_cast<uint64_t>(v) > ma) {
        throw std::runtime_error(
                fmt::format(
                  "Parameter {} value is greater than the maximum "
                  "representable number by an int64 : {} > {}",
                  prmName, v, ma));
      }
    }
  } else {
    if constexpr (std::is_same_v<T, uint64_t>) {
      const auto ma = static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
      if (v > ma) {
        throw std::runtime_error(
                fmt::format(
                  "Parameter {} value is greater than the maximum "
                  "representable number by an int64 : {} > {}",
                  prmName, v, ma));
      }
    }
  }
  static_cast<void>(prmName);
}


template<typename MemberType>
void read_param_to_field(const rclcpp::Parameter & param, MemberType & field)
{
  if constexpr (is_std_vector_v<MemberType>)
  {
    // vector types
    using ElType = typename MemberType::value_type;
    if constexpr (std::is_floating_point_v<ElType>) {
      field = param.as_double_array();
    } else if constexpr (std::is_same_v<ElType, bool>) {
      field = param.as_bool_array();
    } else if constexpr (std::is_integral_v<ElType>) {    // NOLINT
      const auto arr = param.as_integer_array();
      field.clear();
      field.reserve(arr.size());
      for (const auto & v : arr) {
        param_details::check_representable<ElType>(v, param.get_name());
        field.push_back(static_cast<ElType>(v));
      }
    } else if constexpr (is_scoped_enum_v<ElType>) {       // NOLINT
      const auto arr = param.as_integer_array();
      field.clear();
      field.reserve(arr.size());
      for (const auto & v : arr) {
        param_details::check_representable<std::underlying_type_t<ElType>>(v, param.get_name());
        if (v < static_cast<int64_t>(ElType::N_ELEMENTS)) {
          field.push_back(static_cast<ElType>(v));
        } else {
          field.push_back(ElType::UNKNOWN);
        }
      }
    } else if constexpr (std::is_same_v<ElType, std::string>) {
      field = param.as_string_array();
    } else {
      static_assert(
        is_sequence<ElType>::value || std::is_floating_point_v<ElType>||
        std::is_integral_v<ElType>|| is_scoped_enum_v<ElType>||
        std::is_same_v<ElType, std::string>,
        "ROS parameter vector must contain floating-, integral-,"
        " scoped-enum-, string-like elements"
      );
    }
  } else if constexpr (is_std_array_v<MemberType>) {    // NOLINT
    auto check_size = [&](auto v) {
        if (v.size() != field.size()) {
          throw std::runtime_error(
                  fmt::format(
                    "Parameter {} array of size {} does not fit in {} elements",
                    param.get_name(), field.size(), v.size()));
        }
      };
    // array types
    using ElType = typename MemberType::value_type;
    if constexpr (std::is_floating_point_v<ElType>) {
      const auto tmp = param.as_double_array();
      check_size(tmp);
      std::copy(tmp.cbegin(), tmp.cend(), field.begin());
    } else if constexpr (std::is_same_v<ElType, bool>) {    // NOLINT
      const auto tmp = param.as_bool_array();
      check_size(tmp);
      std::copy(tmp.cbegin(), tmp.cend(), field.begin());
    } else if constexpr (std::is_integral_v<ElType>) {    //NOLINT
      const auto tmp = param.as_integer_array();
      check_size(tmp);
      for (const auto & v : tmp) {
        param_details::check_representable<ElType>(v, param.get_name());
      }
      std::copy(tmp.cbegin(), tmp.cend(), field.begin());
    } else if constexpr (is_scoped_enum_v<ElType>) {       // NOLINT
      const auto tmp = param.as_integer_array();
      check_size(tmp);
      for (std::size_t i = 0; i < tmp.size(); i++) {
        const auto & v = tmp[i];
        param_details::check_representable<std::underlying_type_t<ElType>>(v, param.get_name());
        if (v < static_cast<int64_t>(ElType::N_ELEMENTS)) {
          field[i] = static_cast<ElType>(v);
        } else {
          field[i] = ElType::UNKNOWN;
        }
      }
    } else if constexpr (std::is_same_v<ElType, std::string>) {    // NOLINT
      const auto tmp = param.as_string_array();
      check_size(tmp);
      std::copy(tmp.cbegin(), tmp.cend(), field.begin());
    } else {
      static_assert(
        is_sequence<ElType>::value || std::is_floating_point_v<ElType>||
        std::is_integral_v<ElType>|| is_scoped_enum_v<ElType>||
        std::is_same_v<ElType, std::string>,
        "ROS parameter vector must contain floating-, integral-,"
        " scoped-enum-, string-like elements"
      );
    }
  } else {
    // basic types
    if constexpr (std::is_floating_point_v<MemberType>) {    // NOLINT
      field = param.as_double();
    } else if constexpr (std::is_same_v<MemberType, bool>) {
      // must be before is_integral since is_integral_v<bool> = true
      field = param.as_bool();
    } else if constexpr (std::is_integral_v<MemberType>) {    //NOLINT
      const int64_t v = param.as_int();
      param_details::check_representable<MemberType>(v, param.get_name());
      field = v;
    } else if constexpr (is_scoped_enum_v<MemberType>) {       // NOLINT
      const int64_t v = param.as_int();
      param_details::check_representable<std::underlying_type_t<MemberType>>(v, param.get_name());
      if (v < static_cast<int64_t>(MemberType::N_ELEMENTS)) {
        field = static_cast<MemberType>(v);
      } else {
        field = MemberType::UNKNOWN;
      }
    } else if constexpr (std::is_same_v<MemberType, std::string>) {    // NOLINT
      field = param.as_string();
    } else {
      static_assert(
        is_sequence<MemberType>::value ||
        std::is_floating_point_v<MemberType>||
        std::is_integral_v<MemberType>||
        is_scoped_enum_v<MemberType>||
        std::is_same_v<MemberType, std::string>,
        "ROS parameter must be floating-, integral-,"
        " scoped-enum-, string-like, or a vector of the above"
      );
    }
  }
}

}  // namespace param_details


template<typename Seq>
struct ROSDeleter
{
  explicit ROSDeleter(rclcpp::Node & node, const Seq & seq, std::string prefix = "")
  : n_(node), seq_(seq), prefix_(std::move(prefix)) {}

  template<typename Index>
  void operator()(Index)
  {
    using TypeAtIndex = typename boost::fusion::result_of::value_at<Seq, Index>::type;
    const std::string name = struct_member_name<Seq, Index::value>::call();
    const std::string pfx_name = prefix_.empty() ? name : prefix_ + '.' + name;
    auto & field = boost::fusion::at<Index>(seq_);

    if constexpr (is_sequence<TypeAtIndex>::value) {
      using Idx = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<TypeAtIndex>::value>;
      boost::mpl::for_each<Idx>(ROSDeleter<TypeAtIndex>(n_, field, pfx_name));
    } else {
      n_.undeclare_parameter(pfx_name);
    }
  }

private:
  rclcpp::Node & n_;
  const Seq & seq_;
  std::string prefix_;
};


template<typename Seq>
struct ROSReader
{
  explicit ROSReader(const rclcpp::Node & node, Seq & seq, std::string prefix = "")
  : n_(node), seq_(seq), prefix_(std::move(prefix)) {}

  template<typename Index>
  void operator()(Index)
  {
    using TypeAtIndex = typename boost::fusion::result_of::value_at<Seq, Index>::type;
    const std::string name = struct_member_name<Seq, Index::value>::call();
    const std::string pfx_name = prefix_.empty() ? name : prefix_ + '.' + name;
    auto & field = boost::fusion::at<Index>(seq_);

    if constexpr (is_sequence<TypeAtIndex>::value) {
      using Idx = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<TypeAtIndex>::value>;
      boost::mpl::for_each<Idx>(ROSReader<TypeAtIndex>(n_, field, pfx_name));
    } else {
      param_details::read_param_to_field<TypeAtIndex>(n_.get_parameter(pfx_name), field);
    }
  }

private:
  const rclcpp::Node & n_;
  Seq & seq_;
  std::string prefix_;
};


template<typename Seq>
struct ROSDeclarer
{
  explicit ROSDeclarer(rclcpp::Node & node, const Seq & seq, std::string prefix = "")
  : n_(node), seq_(seq), prefix_(std::move(prefix)) {}

  template<typename Index>
  void operator()(Index)
  {
    using TypeAtIndex = typename boost::fusion::result_of::value_at<Seq, Index>::type;
    const std::string name = struct_member_name<Seq, Index::value>::call();
    const std::string pfx_name = prefix_.empty() ? name : prefix_ + '.' + name;
    const auto & field = boost::fusion::at<Index>(seq_);

    if constexpr (is_sequence<TypeAtIndex>::value) {
      using Idx = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<TypeAtIndex>::value>;
      boost::mpl::for_each<Idx>(ROSDeclarer<TypeAtIndex>(n_, field, pfx_name));
      return;
    }

    if constexpr (is_std_vector_v<TypeAtIndex>|| is_std_array_v<TypeAtIndex>) {
      // vector and array types
      using ElementType = typename TypeAtIndex::value_type;
      if constexpr (std::is_floating_point_v<ElementType>) {
        n_.declare_parameter<std::vector<double>>(
          pfx_name, std::vector<double>(field.begin(), field.end())
        );
      } else if constexpr (std::is_same_v<ElementType, bool>) {   // NOLINT
        n_.declare_parameter<std::vector<bool>>(
          pfx_name, std::vector<bool>(field.begin(), field.end())
        );
      } else if constexpr (std::is_integral_v<ElementType>|| is_scoped_enum_v<ElementType>) {   // NOLINT
        std::vector<int64_t> tmp;
        tmp.reserve(field.size());
        for (const auto & v : field) {
          param_details::check_fit_int64(v, pfx_name);
          tmp.emplace_back(static_cast<int64_t>(v));
        }
        n_.declare_parameter<std::vector<int64_t>>(pfx_name, tmp);
      } else if constexpr (std::is_same_v<ElementType, std::string>) {   // NOLINT
        n_.declare_parameter<std::vector<std::string>>(
          pfx_name, std::vector<std::string>(field.begin(), field.end())
        );
      } else {
        static_assert(
          is_sequence<ElementType>::value ||
          std::is_floating_point_v<ElementType>||
          std::is_integral_v<ElementType>||
          is_scoped_enum_v<ElementType>||
          std::is_same_v<ElementType, std::string>,
          "ROS parameter vector must contain floating-, integral-,"
          " scoped-enum-, string-like elements"
        );
      }
    } else {
      // basic types
      if constexpr (std::is_floating_point_v<TypeAtIndex>) {
        n_.declare_parameter<double>(pfx_name, field);
      } else if constexpr (std::is_same_v<TypeAtIndex, bool>) {   // NOLINT
        // must be before is_integral since is_integral_v<bool> = true
        n_.declare_parameter<bool>(pfx_name, field);
      } else if constexpr (std::is_integral_v<TypeAtIndex>|| is_scoped_enum_v<TypeAtIndex>) {   // NOLINT
        const auto & v = field;
        param_details::check_fit_int64(v, pfx_name);
        n_.declare_parameter<int64_t>(pfx_name, static_cast<int64_t>(v));
      } else if constexpr (std::is_same_v<TypeAtIndex, std::string>) {   // NOLINT
        n_.declare_parameter<std::string>(pfx_name, field);
      } else {
        static_assert(
          is_sequence<TypeAtIndex>::value ||
          std::is_floating_point_v<TypeAtIndex>||
          std::is_integral_v<TypeAtIndex>||
          is_scoped_enum_v<TypeAtIndex>||
          std::is_same_v<TypeAtIndex, std::string>,
          "ROS parameter must be floating-, integral-,"
          " scoped-enum-, string-like, or a vector of the above"
        );
      }
    }
  }

private:
  rclcpp::Node & n_;
  const Seq & seq_;
  std::string prefix_;
};


template<typename Seq>
struct ROSParamUpdater
{
  explicit ROSParamUpdater(
    Seq & seq, const rclcpp::Parameter & prm, bool & res, std::string prefix = "")
  : seq_(seq), prm_(prm), res_(res), prefix_(std::move(prefix))
  {}

  template<typename Index>
  void operator()(Index)
  {
    if (res_) {
      return;  // parameter already found
    }

    using TypeAtIndex = typename boost::fusion::result_of::value_at<Seq, Index>::type;
    const std::string name = struct_member_name<Seq, Index::value>::call();
    const std::string pfx_name = prefix_.empty() ? name : prefix_ + '.' + name;
    auto & field = boost::fusion::at<Index>(seq_);

    if constexpr (is_sequence<TypeAtIndex>::value) {
      if (prm_.get_name().substr(0, pfx_name.size()) == pfx_name) {
        using Idx = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<TypeAtIndex>::value>;
        boost::mpl::for_each<Idx>(ROSParamUpdater<TypeAtIndex>(field, prm_, res_, pfx_name));
      }
    } else {
      if (prm_.get_name() == pfx_name) {
        res_ = true;
        param_details::read_param_to_field(prm_, field);
      }
    }
  }

private:
  Seq & seq_;
  const rclcpp::Parameter & prm_;
  bool & res_;
  const std::string prefix_;
};


template<typename Seq>
struct ROSSetter
{
  explicit ROSSetter(rclcpp::Node & node, const Seq & seq, std::string prefix = "")
  : n_(node), seq_(seq), prefix_(std::move(prefix)) {}

  template<typename Index>
  void operator()(Index)
  {
    using TypeAtIndex = typename boost::fusion::result_of::value_at<Seq, Index>::type;
    const std::string name = struct_member_name<Seq, Index::value>::call();
    const std::string pfx_name = prefix_.empty() ? name : prefix_ + '.' + name;
    auto & field = boost::fusion::at<Index>(seq_);

    if constexpr (is_sequence<TypeAtIndex>::value) {
      using Idx = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<TypeAtIndex>::value>;
      boost::mpl::for_each<Idx>(ROSSetter<TypeAtIndex>(n_, field, pfx_name));
      return;
    }

    if constexpr (is_std_vector_v<TypeAtIndex>|| is_std_array_v<TypeAtIndex>)
    {
      // vector and array types
      using ElementType = typename TypeAtIndex::value_type;
      if constexpr (std::is_floating_point_v<ElementType>) {
        n_.set_parameter(
          rclcpp::Parameter(pfx_name, std::vector<double>(field.begin(), field.end()))
        );
      } else if constexpr (std::is_same_v<ElementType, bool>) {   // NOLINT
        n_.set_parameter(
          rclcpp::Parameter(pfx_name, std::vector<bool>(field.begin(), field.end()))
        );
      } else if constexpr (std::is_integral_v<ElementType>|| is_scoped_enum_v<ElementType>) {   // NOLINT
        std::vector<int64_t> tmp;
        tmp.reserve(field.size());
        for (const auto & v : field) {
          param_details::check_fit_int64(v, pfx_name);
          tmp.emplace_back(static_cast<int64_t>(v));
        }
        n_.set_parameter(rclcpp::Parameter(pfx_name, tmp));
      } else if constexpr (std::is_same_v<ElementType, std::string>) {   // NOLINT
        n_.set_parameter(
          rclcpp::Parameter(pfx_name, std::vector<std::string>(field.begin(), field.end()))
        );
      } else {
        static_assert(
          is_sequence<ElementType>::value ||
          std::is_floating_point_v<ElementType>||
          std::is_integral_v<ElementType>||
          std::is_same_v<ElementType, std::string>,
          "ROS parameter vector must contain floating-, integral-, string-like elements"
        );
      }
    } else {
      // basic types
      if constexpr (std::is_floating_point_v<TypeAtIndex>) {
        n_.set_parameter(rclcpp::Parameter(pfx_name, static_cast<double>(field)));
      } else if constexpr (std::is_same_v<TypeAtIndex, bool>) {   // NOLINT
        // must be before is_integral since is_integral_v<bool> = true
        n_.set_parameter(rclcpp::Parameter(pfx_name, field));
      } else if constexpr (std::is_integral_v<TypeAtIndex>|| is_scoped_enum_v<TypeAtIndex>) {   //NOLINT
        const auto & v = field;
        param_details::check_fit_int64(v, pfx_name);
        n_.set_parameter(rclcpp::Parameter(pfx_name, static_cast<int64_t>(v)));
      } else if constexpr (std::is_same_v<TypeAtIndex, std::string>) {   // NOLINT
        n_.set_parameter(rclcpp::Parameter(pfx_name, field));
      } else {
        static_assert(
          is_sequence<TypeAtIndex>::value ||
          std::is_floating_point_v<TypeAtIndex>||
          std::is_integral_v<TypeAtIndex>||
          is_scoped_enum_v<TypeAtIndex>||
          std::is_same_v<TypeAtIndex, std::string>,
          "ROS parameter must be floating-, integral-, scoped-enum-,"
          " string-like, or a vector of the above"
        );
      }
    }
  }

private:
  rclcpp::Node & n_;
  const Seq & seq_;
  std::string prefix_;
};


/* Read ROS parameters into a struct */
template<typename Seq>
Seq readParamsFromNode(
  const rclcpp::Node & node,
  const std::string & prefix = "",
  const bool check_correctness = true)
{
  Seq seq;
  if constexpr (is_sequence<Seq>::value) {
    using Indices = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<Seq>::value>;
    boost::mpl::for_each<Indices>(ROSReader<Seq>(node, seq, prefix));
    if constexpr (is_detected_v<param_details::has_check_correctness_t, Seq>) {
      if (check_correctness) {
        try {
          seq.check_correctness();
        } catch (const std::exception & e) {
          throw std::invalid_argument(
                  fmt::format(
                    "Parameter {} did not pass correctness check: \n{}",
                    boost::typeindex::type_id<Seq>().pretty_name(),
                    e.what()));
        }
      }
    }
  } else {
    seq = node.get_parameter(prefix).get_value<Seq>();
  }
  return seq;
}


template<typename Seq>
void readParamsFromNode(
  const rclcpp::Node & node,
  Seq & seq,
  const std::string & prefix = "",
  const bool check_correctness = true)
{
  seq = readParamsFromNode<Seq>(node, prefix, check_correctness);
}


/* Declare struct as ROS parameters in a node */
template<typename Seq>
void declareParamsInNode(
  rclcpp::Node & node,
  const Seq & seq,
  const std::string & prefix = "",
  const bool check_correctness = true)
{
  if constexpr (is_sequence<Seq>::value) {
    if constexpr (is_detected_v<param_details::has_check_correctness_t, Seq>) {
      if (check_correctness) {
        try {
          seq.check_correctness();
        } catch (const std::exception & e) {
          throw std::invalid_argument(
                  fmt::format(
                    "Parameter {} did not pass correctness check: \n{}",
                    boost::typeindex::type_id<Seq>().pretty_name(),
                    e.what()));
        }
      }
    }
    using Indices = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<Seq>::value>;
    boost::mpl::for_each<Indices>(ROSDeclarer<Seq>(node, seq, prefix));
  } else {
    node.declare_parameter(prefix, seq);
  }
}


/* Set parameter values in a node */
template<typename Seq>
void setParamsInNode(
  rclcpp::Node & node,
  const Seq & seq,
  const std::string & prefix = "",
  const bool check_correctness = true)
{
  if constexpr (is_sequence<Seq>::value) {
    if constexpr (is_detected_v<param_details::has_check_correctness_t, Seq>) {
      if (check_correctness) {
        try {
          seq.check_correctness();
        } catch (const std::exception & e) {
          throw std::invalid_argument(
                  fmt::format(
                    "Parameter {} did not pass correctness check: \n{}",
                    boost::typeindex::type_id<Seq>().pretty_name(),
                    e.what()));
        }
      }
    }
    using Indices = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<Seq>::value>;
    boost::mpl::for_each<Indices>(ROSSetter<Seq>(node, seq, prefix));
  } else {
    node.set_parameter(rclcpp::Parameter(prefix, seq));
  }
}


/* Undeclare parameters from a node */
template<typename Seq>
void deleteParamsFromNode(
  rclcpp::Node & node,
  const Seq & seq,
  const std::string & prefix = "")
{
  if constexpr (is_sequence<Seq>::value) {
    using Indices = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<Seq>::value>;
    boost::mpl::for_each<Indices>(ROSDeleter<Seq>(node, seq, prefix));
  } else {
    node.undeclare_parameter(prefix);
  }
}


/* Declare and then read parameter values in a node */
template<typename Seq>
void initParamsInNode(
  rclcpp::Node & node,
  Seq & seq,
  const std::string & prefix = "",
  const bool check_correctness_declare = true,
  const bool check_correctness_read = true)
{
  declareParamsInNode<Seq>(node, seq, prefix, check_correctness_declare);
  seq = readParamsFromNode<Seq>(node, prefix, check_correctness_read);
}


/* Print the contents of a struct to RCLCPP_INFO */
template<typename Seq>
void rclcppLogParams(
  const rclcpp::Logger & logger,
  const Seq & seq,
  const int padding = 0,
  const char paddingChar = ' ')
{
  YAML::Emitter em;
  em << YAML::Node(seq);

  std::vector<std::string> result;
  boost::split(result, em.c_str(), boost::is_any_of("\n"));
  for (const auto & s : result) {
    const std::string ss = std::string(padding, paddingChar) + s;
    RCLCPP_INFO(logger, "%s", ss.c_str());
  }
}


/* Update struct from ROS parameter, return true if parameter found in struct */
template<typename Seq>
bool updateParams(
  Seq & seq,
  const rclcpp::Parameter & param,
  const std::string & prefix = "",
  const bool check_correctness = true)
{
  static_assert(is_sequence<Seq>::value, "Input must be a boost sequence.");

  using Indices = boost::mpl::range_c<int, 0, boost::fusion::result_of::size<Seq>::value>;

  bool res = false;
  if (param.get_name().substr(0, prefix.size()) == prefix) {
    boost::mpl::for_each<Indices>(ROSParamUpdater<Seq>(seq, param, res, prefix));
  }

  if constexpr (is_detected_v<param_details::has_check_correctness_t, Seq>) {
    if (check_correctness) {
      try {
        seq.check_correctness();
      } catch (const std::exception & e) {
        throw std::invalid_argument(
                fmt::format(
                  "Parameter {} did not pass correctness check: \n{}",
                  boost::typeindex::type_id<Seq>().pretty_name(),
                  e.what()));
      }
    }
  }

  return res;
}

}   // namespace atl

#endif  // atl_LIBRARY__PARAMETER_HPP_
