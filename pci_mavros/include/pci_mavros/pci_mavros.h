#ifndef PCI_MAVROS_H_
#define PCI_MAVROS_H_

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "planner_control_interface/pci_manager.h"

namespace explorer {

// PCIManager for an ArduCopter (GUIDED mode) driven through mavros.
//
// Paths from the planner are resampled and followed with a look-ahead
// position + velocity setpoint on mavros/setpoint_raw/local. The node also
// relays mavros' local odometry, re-stamped on the node clock (sim time) and
// re-framed to <world_frame_id>/<body_frame_id>, as "odometry" + TF so the
// planner's TF lookups line up with the Gazebo point clouds.
class PCIMavros : public PCIManager {
 public:
  explicit PCIMavros(const rclcpp::Node::SharedPtr& node);

  bool loadParams(const std::string ns) override;
  bool initialize() override;
  bool initMotion() override;
  bool goToWaypoint(geometry_msgs::msg::Pose& pose) override;
  bool executePath(const std::vector<geometry_msgs::msg::Pose>& path,
                   std::vector<geometry_msgs::msg::Pose>& modified_path,
                   ExecutionPathType path_type = ExecutionPathType::kLocalPath) override;
  void setState(const geometry_msgs::msg::Pose& pose) override;
  void setVelocity(double v) override;
  void setCurrentVelocity(const geometry_msgs::msg::Vector3& vel) override;
  bool planAhead() override { return planner_trigger_lead_time_ > 0.0; }
  void allocateYawAlongPath(std::vector<geometry_msgs::msg::Pose>& path) const override;
  void allocateYawAlongFistSegment(std::vector<geometry_msgs::msg::Pose>& path) const override;
  double getVelocity(ExecutionPathType path_type) override;

 private:
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void stateCallback(const mavros_msgs::msg::State::ConstSharedPtr& msg);
  void controlTimerCallback();
  // Resample to <= step_ spacing, interpolating yaw (shortest way).
  std::vector<geometry_msgs::msg::Pose> resample(const std::vector<geometry_msgs::msg::Pose>& path,
                                                 double step) const;
  void startPath(const std::vector<geometry_msgs::msg::Pose>& path, double v_max);
  bool waitUntilReady(double timeout_s);
  bool armAndTakeoff();
  void publishSetpoint(const geometry_msgs::msg::Pose& pose, const Eigen::Vector3d& vel);
  geometry_msgs::msg::Pose currentPose();

  // Parameters.
  std::string world_frame_id_ = "world";
  std::string body_frame_id_ = "base_link";
  bool publish_tf_ = true;
  bool auto_takeoff_ = true;
  bool init_motion_enable_ = true;
  bool smooth_heading_enable_ = true;
  double init_z_takeoff_ = 2.0;
  double init_x_forward_ = 1.0;
  double v_max_ = 1.0;
  double v_init_max_ = 0.5;
  double v_homing_max_ = 1.5;
  double v_narrow_env_max_ = 0.5;
  double yaw_rate_max_ = 0.5;
  double a_max_ = 1.0;
  double resample_step_ = 0.3;
  double lookahead_dist_ = 1.0;
  double path_end_dist_thr_ = 0.4;
  double path_end_yaw_thr_ = 0.3;
  double planner_trigger_lead_time_ = 0.0;
  double control_rate_ = 20.0;

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Guarded by mutex_ (touched by the PCI loop and this class' callbacks).
  std::mutex mutex_;
  mavros_msgs::msg::State mavros_state_;
  bool odom_ready_ = false;
  geometry_msgs::msg::Pose odom_pose_;
  std::vector<geometry_msgs::msg::Pose> executing_path_;
  size_t progress_ind_ = 0;
  double path_v_max_ = 1.0;
  bool have_setpoint_ = false;
  geometry_msgs::msg::Pose hold_pose_;
};

}  // namespace explorer

#endif
