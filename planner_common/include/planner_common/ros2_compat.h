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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
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

// Sleep on the main node's clock (sim-time aware), like ros::Duration::sleep().
inline void sleep(double seconds) {
  if (seconds <= 0.0) return;
  if (node())
    node()->get_clock()->sleep_for(rclcpp::Duration::from_seconds(seconds));
  else
    rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1e9)));
}

// tf::createQuaternionMsgFromYaw()
inline geometry_msgs::msg::Quaternion quaternionMsgFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2);
  q.w = std::cos(yaw / 2);
  return q;
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

// ROS 1 style service object: srv.request / srv.response.
template <typename SrvT>
struct ServiceCall {
  typename SrvT::Request request;
  typename SrvT::Response response;
};

// Callback group for service clients that are called (blocking) from inside
// other callbacks; requires a MultiThreadedExecutor.
inline rclcpp::CallbackGroup::SharedPtr clientCallbackGroup() {
  static rclcpp::CallbackGroup::SharedPtr group;
  if (!group && node())
    group = node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  return group;
}

template <typename SrvT>
typename rclcpp::Client<SrvT>::SharedPtr createClient(const rclcpp::Node::SharedPtr& n,
                                                      const std::string& name) {
  return n->create_client<SrvT>(name, rclcpp::ServicesQoS(), clientCallbackGroup());
}

// Blocking service call, like ros::ServiceClient::call(). The client must
// belong to a callback group that another executor thread can serve (see
// clientCallbackGroup()), otherwise the response is never processed.
template <typename SrvT>
bool call(const typename rclcpp::Client<SrvT>::SharedPtr& client, ServiceCall<SrvT>& srv,
          std::chrono::duration<double> timeout = std::chrono::seconds(60)) {
  if (!client || !client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(logger(), "Service %s is not available",
                client ? client->get_service_name() : "<null>");
    return false;
  }
  auto request = std::make_shared<typename SrvT::Request>(srv.request);
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(future);
    RCLCPP_WARN(logger(), "Service %s timed out", client->get_service_name());
    return false;
  }
  srv.response = *future.get();
  return true;
}

// Register a ROS 1 style callback: bool Class::cb(Request&, Response&).
template <typename SrvT, typename C>
typename rclcpp::Service<SrvT>::SharedPtr advertiseService(
    const rclcpp::Node::SharedPtr& n, const std::string& name, C* obj,
    bool (C::*cb)(typename SrvT::Request&, typename SrvT::Response&),
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
  return n->create_service<SrvT>(
      name,
      [obj, cb](const std::shared_ptr<typename SrvT::Request> req,
                std::shared_ptr<typename SrvT::Response> res) { (obj->*cb)(*req, *res); },
      rclcpp::ServicesQoS(), group);
}

// Subscribe with a ROS 1 style member callback taking the message by const
// reference or by ConstSharedPtr; the message type comes from the callback.
template <typename M, typename C>
typename rclcpp::Subscription<M>::SharedPtr subscribe(
    const rclcpp::Node::SharedPtr& n, const std::string& topic, size_t depth, void (C::*cb)(const M&),
    C* obj, rclcpp::CallbackGroup::SharedPtr group = nullptr) {
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  return n->create_subscription<M>(
      topic, rclcpp::QoS(depth), [obj, cb](const M& msg) { (obj->*cb)(msg); }, options);
}

template <typename M, typename C>
typename rclcpp::Subscription<M>::SharedPtr subscribe(
    const rclcpp::Node::SharedPtr& n, const std::string& topic, size_t depth,
    void (C::*cb)(const std::shared_ptr<const M>&), C* obj,
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  return n->create_subscription<M>(
      topic, rclcpp::QoS(depth),
      [obj, cb](const std::shared_ptr<const M> msg) { (obj->*cb)(msg); }, options);
}

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
