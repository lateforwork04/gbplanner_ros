#include "pci_mavros/pci_mavros.h"

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "planner_common/ros2_compat.h"

namespace explorer {

namespace {

double getYaw(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion q;
  q.x = quat.x();
  q.y = quat.y();
  q.z = quat.z();
  q.w = quat.w();
  return q;
}

double wrapAngle(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& p) { return {p.x, p.y, p.z}; }

}  // namespace

PCIMavros::PCIMavros(const rclcpp::Node::SharedPtr& node) : PCIManager(node) {
  using std::placeholders::_1;
  // Own groups: odometry / setpoints keep flowing while the PCI loop blocks
  // in a planner service call or in initMotion().
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;

  mavros_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&PCIMavros::odometryCallback, this, _1), options);
  mavros_state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", rclcpp::QoS(10), std::bind(&PCIMavros::stateCallback, this, _1),
      options);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  setpoint_pub_ =
      node_->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("pci_command_path", 1);
  set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(
      "mavros/set_mode", rclcpp::ServicesQoS(), client_group_);
  arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(
      "mavros/cmd/arming", rclcpp::ServicesQoS(), client_group_);
  takeoff_client_ = node_->create_client<mavros_msgs::srv::CommandTOL>(
      "mavros/cmd/takeoff", rclcpp::ServicesQoS(), client_group_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

bool PCIMavros::loadParams(const std::string ns) {
  using planner_common::param::get;
  std::lock_guard<std::mutex> lock(mutex_);
  get(ns + "/world_frame_id", world_frame_id_);
  get(ns + "/body_frame_id", body_frame_id_);
  get(ns + "/publish_tf", publish_tf_);
  get(ns + "/auto_takeoff", auto_takeoff_);
  get(ns + "/init_motion_enable", init_motion_enable_);
  get(ns + "/smooth_heading_enable", smooth_heading_enable_);
  get(ns + "/planner_trigger_lead_time", planner_trigger_lead_time_);
  get(ns + "/init_motion/z_takeoff", init_z_takeoff_);
  get(ns + "/init_motion/x_forward", init_x_forward_);
  get(ns + "/RobotDynamics/v_max", v_max_);
  get(ns + "/RobotDynamics/v_init_max", v_init_max_);
  get(ns + "/RobotDynamics/v_homing_max", v_homing_max_);
  get(ns + "/RobotDynamics/v_narrow_env_max", v_narrow_env_max_);
  get(ns + "/RobotDynamics/yaw_rate_max", yaw_rate_max_);
  get(ns + "/RobotDynamics/a_max", a_max_);
  get(ns + "/path_following/resample_step", resample_step_);
  get(ns + "/path_following/lookahead_dist", lookahead_dist_);
  get(ns + "/path_following/end_dist_thr", path_end_dist_thr_);
  get(ns + "/path_following/end_yaw_thr", path_end_yaw_thr_);
  get(ns + "/path_following/control_rate", control_rate_);
  if (v_max_ <= 0.0 || yaw_rate_max_ <= 0.0 || resample_step_ <= 0.0 || control_rate_ <= 0.0) {
    ROS_ERROR("[PCIMavros]: v_max, yaw_rate_max, resample_step and control_rate must be > 0");
    return false;
  }
  path_v_max_ = v_max_;
  ROS_INFO("[PCIMavros]: frames %s -> %s, v_max %.2f m/s, auto_takeoff %s",
           world_frame_id_.c_str(), body_frame_id_.c_str(), v_max_,
           auto_takeoff_ ? "true" : "false");
  return true;
}

bool PCIMavros::initialize() {
  control_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&PCIMavros::controlTimerCallback, this), group_);
  pci_status_ = PCIStatus::kReady;
  return true;
}

void PCIMavros::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  // mavros stamps with its own (wall) clock and fixed frame names; re-stamp
  // on the sim clock and re-frame for the planner.
  nav_msgs::msg::Odometry odom = *msg;
  odom.header.stamp = node_->get_clock()->now();
  odom.header.frame_id = world_frame_id_;
  odom.child_frame_id = body_frame_id_;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_pose_ = odom.pose.pose;
    odom_ready_ = true;
  }
  odom_pub_->publish(odom);
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = body_frame_id_;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }
}

void PCIMavros::stateCallback(const mavros_msgs::msg::State::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  mavros_state_ = *msg;
}

geometry_msgs::msg::Pose PCIMavros::currentPose() {
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_pose_;
}

void PCIMavros::setState(const geometry_msgs::msg::Pose& pose) { current_pose_ = pose; }

void PCIMavros::setCurrentVelocity(const geometry_msgs::msg::Vector3& vel) {
  current_vel_ << vel.x, vel.y, vel.z;
}

void PCIMavros::setVelocity(double v) {
  if (v > 0.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    v_max_ = v;
  }
}

double PCIMavros::getVelocity(ExecutionPathType path_type) {
  switch (path_type) {
    case ExecutionPathType::kHomingPath:
      return v_homing_max_;
    case ExecutionPathType::kNarrowEnvPath:
      return v_narrow_env_max_;
    default:
      return v_max_;
  }
}

std::vector<geometry_msgs::msg::Pose> PCIMavros::resample(
    const std::vector<geometry_msgs::msg::Pose>& path, double step) const {
  std::vector<geometry_msgs::msg::Pose> out;
  if (path.empty()) return out;
  out.push_back(path.front());
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d p0 = toEigen(path[i - 1].position);
    const Eigen::Vector3d p1 = toEigen(path[i].position);
    const double yaw0 = getYaw(path[i - 1].orientation);
    const double dyaw = wrapAngle(getYaw(path[i].orientation) - yaw0);
    // Enough samples for both the distance and the yaw change.
    const int n = std::max(1, static_cast<int>(std::ceil(std::max(
                                  (p1 - p0).norm() / step, std::abs(dyaw) / 0.2))));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      geometry_msgs::msg::Pose p;
      const Eigen::Vector3d pt = p0 + t * (p1 - p0);
      p.position.x = pt.x();
      p.position.y = pt.y();
      p.position.z = pt.z();
      p.orientation = yawToQuaternion(wrapAngle(yaw0 + t * dyaw));
      out.push_back(p);
    }
  }
  return out;
}

void PCIMavros::startPath(const std::vector<geometry_msgs::msg::Pose>& path, double v_max) {
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = world_frame_id_;
  path_msg.header.stamp = node_->get_clock()->now();
  for (const auto& p : path) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose = p;
    path_msg.poses.push_back(ps);
  }
  path_pub_->publish(path_msg);

  std::lock_guard<std::mutex> lock(mutex_);
  executing_path_ = path;
  progress_ind_ = 0;
  path_v_max_ = v_max;
  force_stop_ = false;
  pci_status_ = PCIStatus::kRunning;
}

bool PCIMavros::executePath(const std::vector<geometry_msgs::msg::Pose>& path,
                            std::vector<geometry_msgs::msg::Pose>& modified_path,
                            ExecutionPathType path_type) {
  if (path.size() <= 1) return false;  // require at least 2 nodes
  std::vector<geometry_msgs::msg::Pose> path_new = path;
  if (path_type != ExecutionPathType::kManualPath && smooth_heading_enable_)
    allocateYawAlongPath(path_new);
  modified_path = path_new;
  startPath(resample(path_new, resample_step_), getVelocity(path_type));
  ROS_INFO("[PCIMavros]: executing path with %zu vertices at %.2f m/s", path.size(),
           getVelocity(path_type));
  return true;
}

bool PCIMavros::goToWaypoint(geometry_msgs::msg::Pose& pose) {
  std::vector<geometry_msgs::msg::Pose> path{currentPose(), pose};
  startPath(resample(path, resample_step_), v_max_);
  return true;
}

void PCIMavros::controlTimerCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!odom_ready_) return;
  if (force_stop_) {
    force_stop_ = false;
    executing_path_.clear();
    hold_pose_ = odom_pose_;
    have_setpoint_ = true;
    pci_status_ = PCIStatus::kError;
  }
  if (pci_status_ != PCIStatus::kRunning || executing_path_.empty()) {
    if (have_setpoint_) publishSetpoint(hold_pose_, Eigen::Vector3d::Zero());
    return;
  }

  const Eigen::Vector3d cur = toEigen(odom_pose_.position);
  const size_t last = executing_path_.size() - 1;
  // Advance past samples already reached.
  while (progress_ind_ < last &&
         (toEigen(executing_path_[progress_ind_].position) - cur).norm() < lookahead_dist_)
    ++progress_ind_;
  const auto& target = executing_path_[progress_ind_];

  // Remaining path length from the target sample.
  double remaining = (toEigen(target.position) - cur).norm();
  for (size_t i = progress_ind_; i < last; ++i)
    remaining += (toEigen(executing_path_[i + 1].position) -
                  toEigen(executing_path_[i].position)).norm();

  if (progress_ind_ == last) {
    const double yaw_err =
        std::abs(wrapAngle(getYaw(target.orientation) - getYaw(odom_pose_.orientation)));
    if (remaining < path_end_dist_thr_ && yaw_err < path_end_yaw_thr_) {
      hold_pose_ = target;
      have_setpoint_ = true;
      executing_path_.clear();
      pci_status_ = PCIStatus::kReady;
      publishSetpoint(hold_pose_, Eigen::Vector3d::Zero());
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS,
                    "PCI: Ready to trigger the planner.");
      return;
    }
  }
  // Re-plan ahead of the end of the path if configured.
  if (planner_trigger_lead_time_ > 0.0 && remaining / path_v_max_ < planner_trigger_lead_time_)
    pci_status_ = PCIStatus::kReady;

  // Velocity feed-forward along the path, ramped down near the end.
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  if (progress_ind_ < last) {
    const Eigen::Vector3d dir = toEigen(executing_path_[progress_ind_ + 1].position) -
                                toEigen(executing_path_[progress_ind_].position);
    if (dir.norm() > 1e-3) {
      const double speed = std::min(path_v_max_, std::sqrt(2.0 * a_max_ * remaining));
      vel = speed * dir.normalized();
    }
  }
  hold_pose_ = target;
  have_setpoint_ = true;
  publishSetpoint(target, vel);
}

void PCIMavros::publishSetpoint(const geometry_msgs::msg::Pose& pose, const Eigen::Vector3d& vel) {
  mavros_msgs::msg::PositionTarget sp;
  sp.header.stamp = node_->get_clock()->now();
  sp.header.frame_id = world_frame_id_;
  // mavros converts this ENU setpoint to LOCAL_NED for the FCU.
  sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  sp.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                 mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                 mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                 mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  sp.position = pose.position;
  sp.velocity.x = vel.x();
  sp.velocity.y = vel.y();
  sp.velocity.z = vel.z();
  sp.yaw = getYaw(pose.orientation);
  setpoint_pub_->publish(sp);
}

bool PCIMavros::waitUntilReady(double timeout_s) {
  const rclcpp::Time start = node_->get_clock()->now();
  while (rclcpp::ok() && pci_status_ == PCIStatus::kRunning) {
    if ((node_->get_clock()->now() - start).seconds() > timeout_s) return false;
    planner_common::sleep(0.1);
  }
  return pci_status_ == PCIStatus::kReady;
}

bool PCIMavros::armAndTakeoff() {
  mavros_msgs::msg::State state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = mavros_state_;
  }
  if (!state.connected) {
    ROS_ERROR("[PCIMavros]: mavros is not connected to the FCU");
    return false;
  }
  if (state.armed) {
    ROS_INFO("[PCIMavros]: already armed, skipping takeoff");
    return true;
  }
  planner_common::ServiceCall<mavros_msgs::srv::SetMode> mode;
  mode.request.custom_mode = "GUIDED";
  if (!planner_common::call(set_mode_client_, mode, std::chrono::seconds(5)) ||
      !mode.response.mode_sent) {
    ROS_ERROR("[PCIMavros]: failed to switch to GUIDED");
    return false;
  }
  planner_common::ServiceCall<mavros_msgs::srv::CommandBool> arm;
  arm.request.value = true;
  bool armed = false;
  for (int attempt = 0; attempt < 10 && !armed; ++attempt) {
    armed = planner_common::call(arming_client_, arm, std::chrono::seconds(5)) &&
            arm.response.success;
    if (!armed) {
      ROS_WARN("[PCIMavros]: arming rejected (pre-arm checks?), retrying");
      planner_common::sleep(2.0);
    }
  }
  if (!armed) return false;
  planner_common::ServiceCall<mavros_msgs::srv::CommandTOL> takeoff;
  takeoff.request.altitude = init_z_takeoff_;
  if (!planner_common::call(takeoff_client_, takeoff, std::chrono::seconds(5)) ||
      !takeoff.response.success) {
    ROS_ERROR("[PCIMavros]: takeoff command rejected");
    return false;
  }
  // Wait for the climb before streaming setpoints.
  const double z0 = currentPose().position.z;
  const rclcpp::Time start = node_->get_clock()->now();
  while (rclcpp::ok() && currentPose().position.z - z0 < 0.9 * init_z_takeoff_) {
    if ((node_->get_clock()->now() - start).seconds() > 30.0) {
      ROS_ERROR("[PCIMavros]: takeoff did not reach %.1f m", init_z_takeoff_);
      return false;
    }
    planner_common::sleep(0.2);
  }
  return true;
}

bool PCIMavros::initMotion() {
  ROS_INFO("[PCIMavros]: Performing initialization motion");
  if (auto_takeoff_ && !armAndTakeoff()) return false;
  if (!init_motion_enable_) return true;

  // Full turn in place to seed the map, then a short step forward.
  const geometry_msgs::msg::Pose start = currentPose();
  const double yaw0 = getYaw(start.orientation);
  std::vector<geometry_msgs::msg::Pose> init_path{start};
  for (int k = 1; k <= 4; ++k) {
    geometry_msgs::msg::Pose p = start;
    p.orientation = yawToQuaternion(wrapAngle(yaw0 + k * M_PI / 2));
    init_path.push_back(p);
  }
  geometry_msgs::msg::Pose fwd = init_path.back();
  fwd.position.x += init_x_forward_ * std::cos(yaw0);
  fwd.position.y += init_x_forward_ * std::sin(yaw0);
  init_path.push_back(fwd);

  startPath(resample(init_path, resample_step_), v_init_max_);
  const bool ok = waitUntilReady(60.0);
  ROS_INFO("[PCIMavros]: initMotion() %s", ok ? "done" : "timed out");
  return ok;
}

void PCIMavros::allocateYawAlongFistSegment(std::vector<geometry_msgs::msg::Pose>& path) const {
  if (path.size() <= 1) return;
  // Heading of the first segment longer than 0.5 m (skip micro-segments).
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d d = toEigen(path[i].position) - toEigen(path[0].position);
    if (d.head<2>().norm() > 0.5) {
      const auto q = yawToQuaternion(std::atan2(d.y(), d.x()));
      for (size_t k = 0; k < i; ++k) path[k].orientation = q;
      break;
    }
  }
}

void PCIMavros::allocateYawAlongPath(std::vector<geometry_msgs::msg::Pose>& path) const {
  if (path.size() <= 1) return;
  // Keep the current heading at the root, then limit the heading change of
  // each segment to what the yaw rate allows during its traversal.
  double yaw_prev = getYaw(current_pose_.orientation);
  path[0].orientation = current_pose_.orientation;
  for (size_t i = 1; i < path.size(); ++i) {
    const double seg_len =
        (toEigen(path[i].position) - toEigen(path[i - 1].position)).norm();
    double yaw_now = getYaw(path[i].orientation);
    const double dyaw = wrapAngle(yaw_now - yaw_prev);
    const double trans_time = seg_len / v_max_;
    if (trans_time < std::abs(dyaw) / yaw_rate_max_) {
      yaw_now = wrapAngle(yaw_prev + trans_time * yaw_rate_max_ * (dyaw >= 0 ? 1.0 : -1.0));
      path[i].orientation = yawToQuaternion(yaw_now);
    }
    yaw_prev = yaw_now;
  }
}

}  // namespace explorer
