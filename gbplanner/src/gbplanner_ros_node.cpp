#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "gbplanner/gbplanner_ros.h"
#include "planner_common/ros2_compat.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "gbplanner_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  planner_common::setNode(node);

  GbplannerRos planner(node);

  // Planner callbacks share the node's default (mutually exclusive) group as
  // in ROS 1; the extra threads serve service clients called from them.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
