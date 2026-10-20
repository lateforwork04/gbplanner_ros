#ifndef PLANNER_COMMON_ROS2_COMPAT_H_
#define PLANNER_COMMON_ROS2_COMPAT_H_

// Thin layer that keeps the ROS 1 era logging / global-parameter / clock call
// sites of the planner working on rclcpp. Every executable must call
// planner_common::setNode() once, right after creating its main node, before
// any parameter is read or anything is logged through these macros.
//
// Parameters: ROS 1 names such as "<node name>/PlanningParams/v_max" map to the
// ROS 2 parameter "PlanningParams.v_max" of that node. The node must be created
// with automatically_declare_parameters_from_overrides(true) so the YAML values
// are visible without declaring each one.

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace planner_common {

namespace detail {
inline rclcpp::Node::SharedPtr& nodeStorage() {
  static rclcpp::Node::SharedPtr node;
  return node;
}
}  // namespace detail

inline void setNode(const rclcpp::Node::SharedPtr& node) { detail::nodeStorage() = node; }

inline const rclcpp::Node::SharedPtr& node() { return detail::nodeStorage(); }

inline rclcpp::Logger logger() {
  return node() ? node()->get_logger() : rclcpp::get_logger("gbplanner");
}

// Clock of the main node, so use_sim_time applies everywhere.
inline rclcpp::Time now() {
  return node() ? node()->get_clock()->now() : rclcpp::Clock(RCL_ROS_TIME).now();
}

inline rclcpp::Clock& clock() {
  static rclcpp::Clock steady(RCL_STEADY_TIME);
  return node() ? *node()->get_clock() : steady;
}

// Equivalent of ros::this_node::getName(): "/<namespace>/<node>".
inline std::string nodeName() {
  return node() ? node()->get_fully_qualified_name() : std::string("/gbplanner");
}

namespace param {

// "/ns/node/Group/name" -> "Group.name" (relative to the main node).
inline std::string toRos2Name(const std::string& name) {
  std::string n = name;
  const std::string self = nodeName();
  if (n.compare(0, self.size(), self) == 0) n = n.substr(self.size());
  while (!n.empty() && n.front() == '/') n.erase(0, 1);
  for (auto& c : n)
    if (c == '/') c = '.';
  return n;
}

namespace detail {
template <typename T>
bool assign(const rclcpp::Parameter& p, T& value) {
  using rclcpp::ParameterType;
  const auto t = p.get_type();
  if constexpr (std::is_same_v<T, bool>) {
    if (t != ParameterType::PARAMETER_BOOL) return false;
    value = p.as_bool();
  } else if constexpr (std::is_integral_v<T>) {
    if (t != ParameterType::PARAMETER_INTEGER) return false;
    value = static_cast<T>(p.as_int());
  } else if constexpr (std::is_floating_point_v<T>) {
    // ROS 1 accepted "5" for a double; keep that.
    if (t == ParameterType::PARAMETER_DOUBLE)
      value = static_cast<T>(p.as_double());
    else if (t == ParameterType::PARAMETER_INTEGER)
      value = static_cast<T>(p.as_int());
    else
      return false;
  } else if constexpr (std::is_same_v<T, std::string>) {
    if (t != ParameterType::PARAMETER_STRING) return false;
    value = p.as_string();
  } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    if (t != ParameterType::PARAMETER_STRING_ARRAY) return false;
    value = p.as_string_array();
  } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
    if (t != ParameterType::PARAMETER_BOOL_ARRAY) return false;
    value = p.as_bool_array();
  } else {
    using E = typename T::value_type;
    static_assert(std::is_arithmetic_v<E>, "unsupported parameter type");
    value.clear();
    if (t == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if constexpr (std::is_integral_v<E>) return false;
      for (double v : p.as_double_array()) value.push_back(static_cast<E>(v));
    } else if (t == ParameterType::PARAMETER_INTEGER_ARRAY) {
      for (int64_t v : p.as_integer_array()) value.push_back(static_cast<E>(v));
    } else {
      return false;
    }
  }
  return true;
}
}  // namespace detail

// Same contract as ros::param::get: false (value untouched) when unset or of
// the wrong type.
template <typename T>
bool get(const std::string& name, T& value) {
  if (!node()) return false;
  rclcpp::Parameter p;
  if (!node()->get_parameter(toRos2Name(name), p)) return false;
  T tmp{};
  if (!detail::assign(p, tmp)) {
    RCLCPP_WARN(logger(), "Parameter %s has an unexpected type (%s)", name.c_str(),
                p.get_type_name().c_str());
    return false;
  }
  value = tmp;
  return true;
}

}  // namespace param
}  // namespace planner_common

// ROS 1 logging macros on top of rclcpp, logging to the main node's logger.
#define ROS_DEBUG(...) RCLCPP_DEBUG(planner_common::logger(), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(planner_common::logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(planner_common::logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(planner_common::logger(), __VA_ARGS__)
#define ROS_INFO_COND(cond, ...) \
  RCLCPP_INFO_EXPRESSION(planner_common::logger(), (cond), __VA_ARGS__)
#define ROS_WARN_COND(cond, ...) \
  RCLCPP_WARN_EXPRESSION(planner_common::logger(), (cond), __VA_ARGS__)
#define ROS_ERROR_COND(cond, ...) \
  RCLCPP_ERROR_EXPRESSION(planner_common::logger(), (cond), __VA_ARGS__)
#define ROS_INFO_STREAM(args) RCLCPP_INFO_STREAM(planner_common::logger(), args)
#define ROS_WARN_STREAM(args) RCLCPP_WARN_STREAM(planner_common::logger(), args)
#define ROS_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(planner_common::logger(), args)
// ROS 1 throttle periods are in seconds.
#define ROS_INFO_THROTTLE(period, ...)                                       \
  RCLCPP_INFO_THROTTLE(planner_common::logger(), planner_common::clock(), \
                       static_cast<int64_t>((period) * 1000), __VA_ARGS__)
#define ROS_WARN_THROTTLE(period, ...)                                       \
  RCLCPP_WARN_THROTTLE(planner_common::logger(), planner_common::clock(), \
                       static_cast<int64_t>((period) * 1000), __VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...)                                       \
  RCLCPP_ERROR_THROTTLE(planner_common::logger(), planner_common::clock(), \
                        static_cast<int64_t>((period) * 1000), __VA_ARGS__)

#endif  // PLANNER_COMMON_ROS2_COMPAT_H_
