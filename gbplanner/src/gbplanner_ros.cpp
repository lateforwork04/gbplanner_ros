#include <ament_index_cpp/get_package_share_directory.hpp>

#include "gbplanner/gbplanner_ros.h"

GbplannerRos::GbplannerRos(const rclcpp::Node::SharedPtr& node)
  : node_(node)
{
  planner_service_ = planner_common::advertiseService<planner_msgs::srv::PlannerSrv>(node_, "gbplanner_ros", this, &GbplannerRos::plannerServiceCallback);
  
  planner_homing_service_ = planner_common::advertiseService<planner_msgs::srv::PlannerHoming>(node_, "gbplanner_ros_homing", this, &GbplannerRos::plannerHomingServiceCallback);
  
  gbplanner_.reset(new Gbplanner(node_));

  registerTree();
}

bool GbplannerRos::plannerServiceCallback(planner_msgs::srv::PlannerSrv::Request& req,
                              planner_msgs::srv::PlannerSrv::Response& res)
{
  gbplanner_->setPlannerSrvReq(req);
  
  tree_.tickOnce();

  gbplanner_->getPlannerSrvRes(res);
  std::cout << "Sending response to PCI: " << res.status << std::endl;

  return true;
}

bool GbplannerRos::plannerHomingServiceCallback(planner_msgs::srv::PlannerHoming::Request& req,
                              planner_msgs::srv::PlannerHoming::Response& res)
{
  ROS_WARN("Homing through BT");
  gbplanner_->bt_states_.homing_required = true;

  // planner_msgs::srv::PlannerSrv::Request req_planner;
  // planner_msgs::srv::PlannerSrv::Response res_planner;
  // req_planner.header = req.header;
  // req_planner.bound_mode = planner_msgs::msg::BoundMode::EXTENDED_BOUND;

  // gbplanner_->setPlannerSrvReq(req_planner);
  // tree_.tickOnce();
  // gbplanner_->getPlannerSrvRes(res_planner);

  // res.path = res_planner.path;

  return true;
}

void GbplannerRos::registerTree()
{
  factory_.registerNodeType<LocalExploration>("LocalExploration", gbplanner_);
  factory_.registerNodeType<GlobalExploration>("GlobalExploration", gbplanner_);
  factory_.registerNodeType<LocalExpExhaustedCheck>("LocalExpExhaustedCheck", gbplanner_);
  factory_.registerNodeType<GlobalExpExhaustedCheck>("GlobalExpExhaustedCheck", gbplanner_);
  factory_.registerNodeType<Inspection>("Inspection", gbplanner_);
  factory_.registerNodeType<CompartmentTransition>("CompartmentTransition", gbplanner_);
  factory_.registerNodeType<Homing>("Homing", gbplanner_);
  factory_.registerNodeType<HomingCheck>("HomingCheck", gbplanner_);
  factory_.registerNodeType<OPENINGPhase1>("OPENINGPhase1", gbplanner_);
  factory_.registerNodeType<OPENINGPhaseCheck>("OPENINGPhaseCheck", gbplanner_);
  factory_.registerNodeType<OPENINGPhase2>("OPENINGPhase2", gbplanner_);
  factory_.registerNodeType<LocalExpExhaustedReset>("LocalExpExhaustedReset", gbplanner_);
  factory_.registerNodeType<Idle>("Idle", gbplanner_);
  factory_.registerNodeType<OPENINGP1FailCheck>("OPENINGP1FailCheck", gbplanner_);
  factory_.registerNodeType<SetNextCompartment>("SetNextCompartment", gbplanner_);
  factory_.registerNodeType<AllCompartmentsInspectedCheck>("AllCompartmentsInspectedCheck", gbplanner_);
  factory_.registerNodeType<LocalNavigation>("LocalNavigation", gbplanner_);
  factory_.registerNodeType<LocalNavigationExhaustedCheck>("LocalNavigationExhaustedCheck", gbplanner_);
  factory_.registerNodeType<LocalNavigationExhaustedReset>("LocalNavigationExhaustedReset", gbplanner_);
  factory_.registerNodeType<CalculateHomingPath>("CalculateHomingPath", gbplanner_);
  factory_.registerNodeType<UpdateHomingGoal>("UpdateHomingGoal", gbplanner_);
  factory_.registerNodeType<SwitchToLocalNavigation>("SwitchToLocalNavigation", gbplanner_);
  factory_.registerNodeType<CalculateGlobalPath>("CalculateGlobalPath", gbplanner_);
  factory_.registerNodeType<UpdateGlobalGoal>("UpdateGlobalGoal", gbplanner_);

  std::string tree_path;
  if(!planner_common::param::get(planner_common::nodeName() + "/behavior_tree_path", tree_path))
  {
    tree_path = ament_index_cpp::get_package_share_directory("gbplanner") +
                "/config/bt_xml/main_tree.xml";
  }

  ROS_WARN_STREAM("Tree path: " << tree_path);
  
  factory_.registerBehaviorTreeFromFile(tree_path);
  tree_ = factory_.createTree("MainTree");
	std::cout << "Behavior Tree built" << std::endl;

  std::string xml_models = BT::writeTreeNodesModelXML(factory_);

  std::cout << "TreeNodesMode: " << xml_models << std::endl;

  // BT::Groot2Publisher publisher(tree_);
}
