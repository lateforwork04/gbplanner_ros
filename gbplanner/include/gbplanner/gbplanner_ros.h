#pragma once

#include "gbplanner/gbplanner.h"
#include "gbplanner/gbplanner_bt_nodes.h"

class GbplannerRos
{
private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::ServiceBase::SharedPtr planner_service_;
  rclcpp::ServiceBase::SharedPtr planner_homing_service_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<Gbplanner> gbplanner_;
public:
  GbplannerRos(const rclcpp::Node::SharedPtr& node);
  bool plannerServiceCallback(planner_msgs::srv::PlannerSrv::Request& req,
                              planner_msgs::srv::PlannerSrv::Response& res);
  bool plannerHomingServiceCallback(planner_msgs::srv::PlannerHoming::Request& req,
                              planner_msgs::srv::PlannerHoming::Response& res);

  void registerTree();

};
