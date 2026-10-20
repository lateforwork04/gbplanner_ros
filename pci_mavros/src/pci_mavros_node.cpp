#include <rclcpp/rclcpp.hpp>

#include "pci_mavros/pci_mavros.h"
#include "planner_common/ros2_compat.h"
#include "planner_control_interface/planner_control_interface.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "pci_mavros_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  planner_common::setNode(node);

  auto pci_manager = std::make_shared<explorer::PCIMavros>(node);
  // Blocks until the planner services are up, then runs from a timer.
  explorer::PlannerControlInterface interface(node, pci_manager);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
