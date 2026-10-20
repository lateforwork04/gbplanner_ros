
#include "planner_control_interface/planner_control_interface.h"

#include <chrono>
#include <thread>

#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace explorer {

PlannerControlInterface::PlannerControlInterface(
    const rclcpp::Node::SharedPtr& node, std::shared_ptr<PCIManager> pci_manager)
    : node_(node) {
  using std::placeholders::_1;
  main_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = main_group_;
  auto client = [this](auto* tag, const std::string& name) {
    using SrvT = std::remove_pointer_t<decltype(tag)>;
    return node_->create_client<SrvT>(name, rclcpp::ServicesQoS(), client_group_);
  };
  auto service = [this](auto* tag, const std::string& name, auto cb) {
    using SrvT = std::remove_pointer_t<decltype(tag)>;
    return planner_common::advertiseService<SrvT>(node_, name, this, cb, main_group_);
  };
  using namespace planner_msgs::srv;
  using std_srvs::srv::Trigger;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  reference_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("ref_pose", 5);
  planner_status_pub_ = node_->create_publisher<std_msgs::msg::Bool>("gbplanner_status", 5);
  stop_request_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "planner_control_interface/stop_request", 5);

  odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 1, std::bind(&PlannerControlInterface::odometryCallback, this, std::placeholders::_1),
      sub_options);
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", 1, std::bind(&PlannerControlInterface::poseCallback, this, std::placeholders::_1), sub_options);
  pose_stamped_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_stamped", 1, std::bind(&PlannerControlInterface::poseStampedCallback, this, std::placeholders::_1),
      sub_options);

  pci_server_ = service((PciTrigger*)nullptr, "planner_control_interface_trigger",
                        &PlannerControlInterface::triggerCallback);
  pci_std_automatic_planning_server_ =
      service((Trigger*)nullptr, "planner_control_interface/std_srvs/automatic_planning",
              &PlannerControlInterface::stdSrvsAutomaticPlanningCallback);
  pci_std_single_planning_server_ =
      service((Trigger*)nullptr, "planner_control_interface/std_srvs/single_planning",
              &PlannerControlInterface::stdSrvsSinglePlanningCallback);
  pci_homing_server_ = service((PciHomingTrigger*)nullptr, "pci_homing_trigger",
                               &PlannerControlInterface::homingCallback);
  pci_std_homing_server_ =
      service((Trigger*)nullptr, "planner_control_interface/std_srvs/homing_trigger",
              &PlannerControlInterface::stdSrvHomingCallback);
  pci_std_go_to_waypoint_server_ =
      service((Trigger*)nullptr, "planner_control_interface/std_srvs/go_to_waypoint",
              &PlannerControlInterface::stdSrvGoToWaypointCallback);
  pci_initialization_server_ =
      service((PciInitialization*)nullptr, "pci_initialization_trigger",
              &PlannerControlInterface::initializationCallback);

  planner_client_ = client((PlannerSrv*)nullptr, "planner_server");
  while (rclcpp::ok() && !planner_client_->wait_for_service(std::chrono::seconds(1))) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "PCI: service planner_server is not available: waiting...");
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "PCI: connected to service planner_server.");
  planner_homing_client_ = client((PlannerHoming*)nullptr, "planner_homing_server");
  while (rclcpp::ok() && !planner_homing_client_->wait_for_service(std::chrono::seconds(1))) {
    ROS_WARN_COND(
        global_verbosity >= Verbosity::WARN,
        "PCI: service planner_homing_server is not available: waiting...");
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "PCI: connected to service planner_homing_server.");

  pci_set_homing_pos_server_ = service((PciSetHomingPos*)nullptr, "pci_set_homing_pos",
                                       &PlannerControlInterface::setHomingPosCallback);
  pci_std_set_homing_pos_server_ = service(
      (Trigger*)nullptr, "planner_control_interface/std_srvs/set_homing_position_here",
      &PlannerControlInterface::stdSrvSetHomingPositionHereCallback);
  planner_set_homing_pos_client_ =
      client((PlannerSetHomingPos*)nullptr, "gbplanner/set_homing_pos");

  planner_set_trigger_mode_client_ =
      client((PlannerSetPlanningMode*)nullptr, "gbplanner/set_planning_trigger_mode");

  pci_search_server_ =
      service((PciSearch*)nullptr, "pci_search", &PlannerControlInterface::searchCallback);
  planner_search_client_ = client((PlannerSearch*)nullptr, "gbplanner/search");

  pci_global_server_ = service((PciGlobal*)nullptr, "pci_global",
                               &PlannerControlInterface::globalPlannerCallback);
  planner_global_client_ = client((PlannerGlobal*)nullptr, "gbplanner/global");
  pci_stop_server_ =
      service((PciStop*)nullptr, "pci_stop", &PlannerControlInterface::stopPlannerCallback);
  pci_std_stop_server_ = service((Trigger*)nullptr, "planner_control_interface/std_srvs/stop",
                                 &PlannerControlInterface::stdSrvStopPlannerCallback);

  planner_geofence_client_ = client((PlannerGeofence*)nullptr, "gbplanner/geofence");
  pci_geofence_server_ = service((PciGeofence*)nullptr, "pci_geofence",
                                 &PlannerControlInterface::addGeofenceCallback);
  pci_to_waypoint_server_ = service((PciToWaypoint*)nullptr, "pci_to_waypoint",
                                    &PlannerControlInterface::goToWaypointCallback);

  planner_passing_gate_client_ =
      client((PlannerRequestPath*)nullptr, "gbplanner/passing_gate");
  pci_passing_gate_server_ =
      service((Trigger*)nullptr, "planner_control_interface/std_srvs/pass_gate",
              &PlannerControlInterface::passingGateCallback);
  pci_inspection_srv_server_ =
      service((Trigger*)nullptr, "planner_control_interface/std_srvs/inspection_srv_trigger",
              &PlannerControlInterface::inspectionSrvCallback);
  planner_inspection_srv_client_ =
      client((PlannerSrv*)nullptr, "gbplanner/get_inspection_path");
  rotate_180_deg_server_ = service((Trigger*)nullptr, "pci_rotate_180_trigger",
                                   &PlannerControlInterface::rotate180DegCallback);

  pci_std_global_last_specified_frontier_server_ = service(
      (Trigger*)nullptr, "planner_control_interface/std_srvs/replan_last_specified_frontier",
      &PlannerControlInterface::stdSrvReplanLastSpecifiedFrontierCallback);

  planner_set_exp_mode_client_ = client((PlannerSetExpMode*)nullptr, "gbplanner/set_exp_mode");

  imarker_server_ =
      std::make_shared<interactive_markers::InteractiveMarkerServer>("waypoints", node_);

  semantic_server =
      std::make_shared<interactive_markers::InteractiveMarkerServer>("semantics", node_);
  semantic_pub = node_->create_publisher<planner_semantic_msgs::msg::SemanticPoint>(
      "semantic_location", 10);

  nav_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1, std::bind(&PlannerControlInterface::navGoalCallback, this, std::placeholders::_1),
      sub_options);
  pose_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/global_planner/waypoint_request", 1,
      std::bind(&PlannerControlInterface::poseGoalCallback, this, std::placeholders::_1), sub_options);
  nav_goal_client_ = client((PlannerGoToWaypoint*)nullptr, "gbplanner/go_to_waypoint");
  go_to_waypoint_visualization_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "gbplanner/go_to_waypoint_pose_visualization", 1);

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "[PCI]: Setting pci_manager_");
  pci_manager_ = pci_manager;

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PCI]: Loading params");
  if (!loadParams()) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR,
                   "Can not load params. Shut down ros node.");
    rclcpp::shutdown();
    return;
  }

  // init() and the planner loop run from this timer (20Hz), on the same
  // callback group as the subscriptions.
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "[PCI]: Starting run() loop");
  run_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                        [this]() { runOnce(); }, main_group_);
}

void PlannerControlInterface::poseGoalCallback(
    const geometry_msgs::msg::PoseStamped& pose_msgs) {
  setGoal(pose_msgs);
}

void PlannerControlInterface::navGoalCallback(
    const geometry_msgs::msg::PoseStamped& nav_msgs) {
  geometry_msgs::msg::PoseStamped posest;
  posest.header = nav_msgs.header;
  posest.pose = nav_msgs.pose;
  setGoal(posest);
}

void PlannerControlInterface::setGoal(const geometry_msgs::msg::PoseStamped& pose) {
  geometry_msgs::msg::PoseStamped pose_in_world_frame;

  try {
    geometry_msgs::msg::TransformStamped to_world = tf_buffer_->lookupTransform(
        world_frame_id_, pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(pose, pose_in_world_frame, to_world);
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR,
                   "[gbplanner_pci::setGoal] %s", ex.what());
    return;
  }

  received_first_waypoint_to_go_ = true;
  go_to_waypoint_with_checking_ = true;
  set_waypoint_stamped_ = pose_in_world_frame;
  ROS_INFO_COND(
      global_verbosity >= Verbosity::INFO,
      "[gbplanner_pci::setGoal] set waypoint to (%.2f, %.2f, %.2f) in frame "
      "'%s'",
      pose_in_world_frame.pose.position.x, pose_in_world_frame.pose.position.y,
      pose_in_world_frame.pose.position.z,
      pose_in_world_frame.header.frame_id.c_str());
  publishGoToWaypointVisualization(set_waypoint_stamped_);
}

bool PlannerControlInterface::passingGateCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  if (!passing_gate_success_) {
    passing_gate_request_ = true;
    return true;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "Passing gate already activated.");
    return false;
  }
}

bool PlannerControlInterface::inspectionSrvCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  //
  if(!inspection_srv_request_) {
    inspection_srv_request_ = true;
    return true;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "Inspection service already activated.");
    return false;
  }
}

void PlannerControlInterface::resetPlanner() {
  // Set back to manual mode, and stop all current requests.
  trigger_mode_ = PlannerTriggerModeType::kManual;
  run_en_ = false;
  search_request_ = false;
  homing_request_ = false;
  init_request_ = false;
  global_request_ = false;
  go_to_waypoint_request_ = false;
  go_to_waypoint_with_checking_ = false;
  inspection_srv_request_ = false;

  // Remove the last waypoint to prevent the planner starts from that last wp.
  current_path_.clear();
  pci_manager_->setStatus(PCIManager::PCIStatus::kReady);
}

bool PlannerControlInterface::rotate180DegCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  go_to_waypoint_request_ = true;
  go_to_waypoint_with_checking_ = false;
  set_waypoint_.position.x = current_pose_.position.x;
  set_waypoint_.position.y = current_pose_.position.y;
  set_waypoint_.position.z = current_pose_.position.z;
  set_waypoint_.orientation.x = 0.0;
  set_waypoint_.orientation.y = 0.0;
  set_waypoint_.orientation.z = 1.0;
  set_waypoint_.orientation.w = 0.0;
  return true;
}

bool PlannerControlInterface::goToWaypointCallback(
    planner_msgs::srv::PciToWaypoint::Request& req,
    planner_msgs::srv::PciToWaypoint::Response& res) {
  planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  go_to_waypoint_request_ = true;
  go_to_waypoint_with_checking_ = false;
  set_waypoint_.position.x = req.waypoint.position.x;
  set_waypoint_.position.y = req.waypoint.position.y;
  set_waypoint_.position.z = req.waypoint.position.z;
  set_waypoint_.orientation.x = req.waypoint.orientation.x;
  set_waypoint_.orientation.y = req.waypoint.orientation.y;
  set_waypoint_.orientation.z = req.waypoint.orientation.z;
  set_waypoint_.orientation.w = req.waypoint.orientation.w;
  return true;
}

bool PlannerControlInterface::searchCallback(
    planner_msgs::srv::PciSearch::Request& req,
    planner_msgs::srv::PciSearch::Response& res) {
  search_request_ = true;
  exe_path_en_ = !req.not_exe_path;
  use_current_state_ = req.use_current_state;
  bound_mode_ = req.bound_mode;
  return true;
}

bool PlannerControlInterface::globalPlannerCallback(
    planner_msgs::srv::PciGlobal::Request& req,
    planner_msgs::srv::PciGlobal::Response& res) {
  global_request_ = true;
  exe_path_en_ = !req.not_exe_path;
  bound_mode_ = req.bound_mode;
  frontier_id_ = req.id;
  pci_global_request_params_ = req;
  res.success = true;
  return true;
}

bool PlannerControlInterface::stopPlannerCallback(
    planner_msgs::srv::PciStop::Request& req,
    planner_msgs::srv::PciStop::Response& res) {
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "[PlannerControlInterface::stopPlannerCallback]");
  std_msgs::msg::Bool stop_msg;
  stop_request_pub_->publish(stop_msg);
  pci_manager_->stopPCI();
  stop_planner_request_ = true;
  resetPlanner();

  // planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  // planning_mode_srv.request.planning_mode =
  //     planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  // planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  res.success = true;
  ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "[PCI] STOP PLANNER.");
  return true;
}

bool PlannerControlInterface::stdSrvStopPlannerCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  planner_msgs::srv::PciStop::Request stop_request;
  planner_msgs::srv::PciStop::Response stop_response;

  res.success = stopPlannerCallback(stop_request, stop_response);
  res.success &= stop_response.success;

  return true;
}

bool PlannerControlInterface::addGeofenceCallback(
    planner_msgs::srv::PciGeofence::Request& req,
    planner_msgs::srv::PciGeofence::Response& res) {
  planner_common::ServiceCall<planner_msgs::srv::PlannerGeofence> plan_srv;
  plan_srv.request.rectangles = req.rectangles;
  if (planner_common::call(planner_geofence_client_, plan_srv))
    res.success = plan_srv.response.success;

  return true;
}

bool PlannerControlInterface::setHomingPosCallback(
    planner_msgs::srv::PciSetHomingPos::Request& req,
    planner_msgs::srv::PciSetHomingPos::Response& res) {
  // Bypass this request to the planner.
  planner_common::ServiceCall<planner_msgs::srv::PlannerSetHomingPos> plan_srv;
  if (planner_common::call(planner_set_homing_pos_client_, plan_srv)) {
    res.success = plan_srv.response.success;
  }
  return true;
}

bool PlannerControlInterface::stdSrvSetHomingPositionHereCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  planner_msgs::srv::PciSetHomingPos::Request set_homing_pos_request;
  planner_msgs::srv::PciSetHomingPos::Response set_homing_pos_response;

  res.success =
      setHomingPosCallback(set_homing_pos_request, set_homing_pos_response);
  res.success &= set_homing_pos_response.success;

  return true;
}

bool PlannerControlInterface::initializationCallback(
    planner_msgs::srv::PciInitialization::Request& req,
    planner_msgs::srv::PciInitialization::Response& res) {
  init_request_ = true;
  res.success = true;
  return true;
}

bool PlannerControlInterface::homingCallback(
    planner_msgs::srv::PciHomingTrigger::Request& req,
    planner_msgs::srv::PciHomingTrigger::Response& res) {
  exe_path_en_ = !req.not_exe_path;
  homing_request_ = true;
  res.success = true;
  return true;
}

bool PlannerControlInterface::stdSrvHomingCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  planner_msgs::srv::PciHomingTrigger::Request homing_trigger_request;
  planner_msgs::srv::PciHomingTrigger::Response homing_trigger_response;

  homing_trigger_request.not_exe_path = false;

  res.success = homingCallback(homing_trigger_request, homing_trigger_response);
  res.success &= homing_trigger_response.success;

  return true;
}

bool PlannerControlInterface::triggerCallback(
    planner_msgs::srv::PciTrigger::Request& req,
    planner_msgs::srv::PciTrigger::Response& res) {
  if (pci_manager_->getStatus() == PCIManager::PCIStatus::kError) {
    ROS_WARN_COND(
        global_verbosity >= Verbosity::WARN,
        "PCIManager is curretely in error state and cannot accept planning "
        "requests.");
    res.success = false;
  } else {
    if ((!req.set_auto) && (trigger_mode_ == PlannerTriggerModeType::kAuto)) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                    "Switch to manual mode.");
      trigger_mode_ = PlannerTriggerModeType::kManual;
    } else if ((req.set_auto) &&
               (trigger_mode_ == PlannerTriggerModeType::kManual)) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                    "Switch to auto mode.");
      trigger_mode_ = PlannerTriggerModeType::kAuto;
    }
    pci_manager_->setVelocity(req.vel_max);
    bound_mode_ = req.bound_mode;
    run_en_ = true;
    exe_path_en_ = !req.not_exe_path;
    res.success = true;
  }
  return true;
}

bool PlannerControlInterface::stdSrvsAutomaticPlanningCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  planner_msgs::srv::PciTrigger::Request pci_trigger_request;
  planner_msgs::srv::PciTrigger::Response pci_trigger_response;
  pci_trigger_request.not_exe_path = false;
  pci_trigger_request.set_auto = true;
  pci_trigger_request.bound_mode = 0;
  pci_trigger_request.vel_max = 0.0;

  res.success = triggerCallback(pci_trigger_request, pci_trigger_response);
  res.success &= pci_trigger_response.success;

  return true;
}

bool PlannerControlInterface::stdSrvGoToWaypointCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  if (received_first_waypoint_to_go_) {
    go_to_waypoint_request_ = true;
    go_to_waypoint_with_checking_ = true;
    res.success = true;
  } else {
    ROS_ERROR_COND(
        global_verbosity >= Verbosity::ERROR,
        "No waypoint was set, 'go_to_waypoint' feature will not be triggered.");
    res.success = false;
  }
  return true;
}

bool PlannerControlInterface::stdSrvsSinglePlanningCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  planner_msgs::srv::PciTrigger::Request pci_trigger_request;
  planner_msgs::srv::PciTrigger::Response pci_trigger_response;
  pci_trigger_request.not_exe_path = false;
  pci_trigger_request.set_auto = false;
  pci_trigger_request.bound_mode = 0;
  pci_trigger_request.vel_max = 0.0;

  res.success = triggerCallback(pci_trigger_request, pci_trigger_response);
  res.success &= pci_trigger_response.success;
  return true;
}

bool PlannerControlInterface::stdSrvReplanLastSpecifiedFrontierCallback(
    std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res) {
  planner_msgs::srv::PciGlobal::Request pci_global_request;
  planner_msgs::srv::PciGlobal::Response pci_global_response;
  pci_global_request.not_exe_path = false;
  pci_global_request.set_auto = false;
  pci_global_request.bound_mode = 0;
  pci_global_request.vel_max = 0.0;
  // Use last frontier specified via service call.
  pci_global_request.id = frontier_id_;

  res.success = globalPlannerCallback(pci_global_request, pci_global_response);
  res.success &= pci_global_response.success;

  return true;
}

bool PlannerControlInterface::init() {
  planner_iteration_ = 0;
  reference_pub_id_ = 0;
  homing_request_ = false;
  run_en_ = false;
  exe_path_en_ = true;
  pose_is_ready_ = false;
  planner_common::ServiceCall<planner_msgs::srv::PlannerSrv> plan_srv_temp;
  bound_mode_ = plan_srv_temp.request.EXTENDED_BOUND;
  force_forward_ = true;
  init_request_ = false;
  search_request_ = false;
  global_request_ = false;
  stop_planner_request_ = false;
  passing_gate_request_ = false;
  passing_gate_success_ = false;
  go_to_waypoint_request_ = false;
  // runOnce() only calls this once odometry is ready.
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[PCI]:[init()]: recieved odom");
  initIMarker();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[PCI]:[init()]: initIMarker");
  initSemanticIMarker();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[PCI]:[init()]: initSemanticIMarker");
  if (!pci_manager_->initialize()) return false;
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[PCI]:[init()]: pci manager init done");
  return true;
}

void PlannerControlInterface::runOnce() {
  if (!initialized_) {
    if (!pose_is_ready_) {
      ROS_WARN_THROTTLE(5.0, "[PCI]: Waiting for odometry.");
      return;
    }
    if (!init()) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR,
                     "Can not initialize the node. Shut down ros node.");
      run_timer_->cancel();
      rclcpp::shutdown();
      return;
    }
    initialized_ = true;
  }
  {
    PCIManager::PCIStatus pci_status = pci_manager_->getStatus();
    // TODO: Fix by prioritizing and sequencing exclusive cases (with bad
    // if/else and flags approach)
    if (pci_status == PCIManager::PCIStatus::kReady) {
      // Priority 1: Check if require homing.
      if (homing_request_) {
        homing_request_ = false;
        trigger_mode_ = PlannerTriggerModeType::kManual;  // also unset auto
                                                          // mode
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "PlannerControlInterface: Running Homing");
        runHoming(exe_path_en_);
      }
      // Priority 2: Check if require initialization step.
      else if (init_request_) {
        init_request_ = false;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "PlannerControlInterface: Running Initialization");
        runInitialization();
        // Priority 3: Stop
      } else if (stop_planner_request_) {
        // Stop at current pose.
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                      "PCI: run: stop requested");
        stop_planner_request_ = false;
        pci_manager_->goToWaypoint(current_pose_);
      } else if ((trigger_mode_ == PlannerTriggerModeType::kAuto) ||
                 (run_en_)) {
        run_en_ = false;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "PlannerControlInterface: Running Planner (%s)",
                      std::string(trigger_mode_ == PlannerTriggerModeType::kAuto
                                      ? "kAuto"
                                      : "kManual")
                          .c_str());
        runPlanner(exe_path_en_);
      } else if (search_request_) {
        search_request_ = false;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "Request the planner to search for connection path.");
        runSearch(exe_path_en_);
      } else if (global_request_) {
        global_request_ = false;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "Request the global planner.");
        runGlobalPlanner(exe_path_en_);
      } else if (passing_gate_request_) {
        passing_gate_request_ = false;
        runPassingGate();
      } else if (inspection_srv_request_) {
        inspection_srv_request_ = false;
        runInspection();
      } else if (go_to_waypoint_request_) {
        go_to_waypoint_request_ = false;
        if (!go_to_waypoint_with_checking_)
          pci_manager_->goToWaypoint(set_waypoint_);
        else
          runGlobalRepositioning();
      }
    } else if (pci_status == PCIManager::PCIStatus::kError) {
      // For ANYmal, reset everything to manual then wait for operator.
      resetPlanner();
    }
  }
}

void PlannerControlInterface::runGlobalRepositioning() {
  ROS_INFO_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Global Repositioning %i",
                planner_iteration_);

  planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  planner_common::ServiceCall<planner_msgs::srv::PlannerGoToWaypoint> planner_srv;
  planner_srv.request.check_collision = true;
  planner_srv.request.waypoint.header = set_waypoint_stamped_.header;
  planner_srv.request.waypoint.pose = set_waypoint_stamped_.pose;

  if (planner_common::call(nav_goal_client_, planner_srv)) {
    if (!planner_srv.response.path.empty()) {
      // Execute path.
      current_path_.clear();
      // resetPlanner();
      std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
      pci_manager_->executePath(planner_srv.response.path, path_to_be_exe,
                                PCIManager::ExecutionPathType::kGlobalPath);
      current_path_ = path_to_be_exe;
    } else {
      ROS_WARN_THROTTLE(1, "Will not execute the path.");
      planner_common::sleep(0.5);
    }
    planner_iteration_++;
  } else {
    ROS_WARN_THROTTLE(1, "Planner service failed");
    planner_common::sleep(0.5);
  }
}

void PlannerControlInterface::runPassingGate() {
  bool success = true;

  planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  planner_common::ServiceCall<planner_msgs::srv::PlannerRequestPath> plan_srv;
  if ((!planner_common::call(planner_passing_gate_client_, plan_srv)) ||
      (plan_srv.response.path.empty()))
    success = false;

  std_msgs::msg::Bool planner_success_msg;
  planner_success_msg.data = success;
  planner_status_pub_->publish(planner_success_msg);

  if (success) {
    passing_gate_success_ = true;
    std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
    pci_manager_->executePath(plan_srv.response.path, path_to_be_exe);
  }
}

void PlannerControlInterface::runInspection() {
  planner_common::ServiceCall<planner_msgs::srv::PlannerSrv> plan_srv;
  plan_srv.request.header.stamp = planner_common::now();
  plan_srv.request.header.frame_id = world_frame_id_;
  plan_srv.request.bound_mode = 0;
  ROS_WARN_COND(global_verbosity >= Verbosity::INFO,"[PCI]: Called inspection srv");
  if (planner_common::call(planner_inspection_srv_client_, plan_srv)) {
    std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
    pci_manager_->executePath(plan_srv.response.path, path_to_be_exe,
                              PCIManager::ExecutionPathType::kManualPath);
    current_path_ = path_to_be_exe;
  }
  ++planner_iteration_;
}

void PlannerControlInterface::runGlobalPlanner(bool exe_path = false) {
  ROS_INFO_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Planning iteration %i",
                planner_iteration_);

  planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  planner_common::ServiceCall<planner_msgs::srv::PlannerGlobal> plan_srv;
  plan_srv.request.id = pci_global_request_params_.id;
  plan_srv.request.not_check_frontier =
      pci_global_request_params_.not_check_frontier;
  plan_srv.request.ignore_time = pci_global_request_params_.ignore_time;
  if (planner_common::call(planner_global_client_, plan_srv)) {
    if ((exe_path) && (!plan_srv.response.path.empty())) {
      // Execute path.
      std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
      pci_manager_->executePath(plan_srv.response.path, path_to_be_exe,
                                PCIManager::ExecutionPathType::kGlobalPath);
      current_path_ = path_to_be_exe;
    } else {
      ROS_WARN_THROTTLE(1, "Will not execute the path.");
      planner_common::sleep(0.5);
    }
    planner_iteration_++;
  } else {
    ROS_WARN_THROTTLE(1, "Planner service failed");
    planner_common::sleep(0.5);
  }
}

void PlannerControlInterface::runPlanner(bool exe_path = false) {
  const int kBBoxLevel = 3;
  bool success = false;

  // planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  // if (trigger_mode_ == PlannerTriggerModeType::kAuto) {
  //   planning_mode_srv.request.planning_mode =
  //       planner_msgs::srv::PlannerSetPlanningMode::Request::AUTO;
  // } else {
  //   planning_mode_srv.request.planning_mode =
  //       planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  // }

  // ROS_WARN("[PCI]: Called trigger mode srv");
  // planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  for (int ind = 0; ind < kBBoxLevel; ++ind) {
    // rclcpp::Duration::from_seconds(0.01)
    //     .sleep();  // sleep to unblock the thread to get latest cmd.
    // ros::spinOnce();
    if (stop_planner_request_) return;

    bound_mode_ = ind;
    ROS_INFO_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Planning iteration %i",
                  planner_iteration_);
    planner_common::ServiceCall<planner_msgs::srv::PlannerSrv> plan_srv;
    plan_srv.request.header.stamp = planner_common::now();
    plan_srv.request.header.frame_id = world_frame_id_;
    plan_srv.request.bound_mode = bound_mode_;
    plan_srv.request.root_pose = getPoseToStart();
    if(ind > 0)
    {
      plan_srv.request.root_pose.position.x = 0.0;
      plan_srv.request.root_pose.position.y = 0.0;
      plan_srv.request.root_pose.position.z = 0.0;
      plan_srv.request.root_pose.orientation.x = 0.0;
      plan_srv.request.root_pose.orientation.y = 0.0;
      plan_srv.request.root_pose.orientation.z = 0.0;
      plan_srv.request.root_pose.orientation.w = 1.0;
    }
    ROS_WARN_COND(global_verbosity >= Verbosity::ERROR,"[PCI]: Called plan srv");
    if (planner_common::call(planner_client_, plan_srv)) {
      if (!plan_srv.response.path.empty()) {
        // Execute path.
        if (exe_path) {
          if ((!force_forward_) ||
              (plan_srv.response.status != plan_srv.response.BACKWARD) ||
              (ind == (kBBoxLevel - 1))) {
            if (ind == (kBBoxLevel - 1))
              ROS_WARN_COND(
                  global_verbosity >= Verbosity::WARN,
                  "Using minimum bound, pick the current best one regardless "
                  "the direction.");
            current_path_.clear();
            std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
            PCIManager::ExecutionPathType path_type =
                PCIManager::ExecutionPathType::kLocalPath;

            ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,"[PCI]: returned status: %d", plan_srv.response.status);
            if (plan_srv.response.status == planner_msgs::srv::PlannerSrv::Response::HOMING) {
              // Perform homing step, set back to manual mode, and stop all
              // current requests.
              resetPlanner();
              path_type = PCIManager::ExecutionPathType::kHomingPath;
            }
            else if (plan_srv.response.status == planner_msgs::srv::PlannerSrv::Response::AUTO_CUSTOM_PATH) {
              // Perform homing step, set back to manual mode, and stop all
              // current requests.
              ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,"[PCI]: Auto Custom Path");
              path_type = PCIManager::ExecutionPathType::kManualPath;
            }
            else if (plan_srv.response.status == planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH) {
              // Perform homing step, set back to manual mode, and stop all
              // current requests.
              ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,"[PCI]: Manual Custom Path");
              resetPlanner();
              path_type = PCIManager::ExecutionPathType::kManualPath;
            }
            else {
              ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,"[PCI]: Local Path");
            }
            v_current_ = pci_manager_->getVelocity(path_type);
            // Publish the status
            publishPlannerStatus(plan_srv.response, true);
            pci_manager_->executePath(plan_srv.response.path, path_to_be_exe,
                                      path_type);
            success = true;
            current_path_ = path_to_be_exe;
          } else if (ind < (kBBoxLevel - 1)) {
            publishPlannerStatus(plan_srv.response, false);
            ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                          "Attemp to re-plan with smaller bound.");
          }
        }
      } else {
        publishPlannerStatus(plan_srv.response, false);
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Planner returned an empty path");
        if (plan_srv.response.status == plan_srv.response.HOMING || plan_srv.response.status == planner_msgs::srv::PlannerSrv::Response::MANUAL_CUSTOM_PATH) {
          // Ran out of time budget or already at home. Stop and reset planner
          resetPlanner();
          success = true;
        }
        // planner_common::sleep(0.5);
      }
      planner_iteration_++;
      if (success) break;
    } else {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Planner service failed");
      planner_common::sleep(0.5);
    }
  }

  // Reset default mode again.
  bound_mode_ = 0;
}

void PlannerControlInterface::publishPlannerStatus(
    const planner_msgs::srv::PlannerSrv::Response& res, bool success) {
  std_msgs::msg::Bool planner_success_msg;
  planner_success_msg.data = success;
  planner_status_pub_->publish(planner_success_msg);
}

void PlannerControlInterface::publishGoToWaypointVisualization(
    const geometry_msgs::msg::PoseStamped& poseStamped) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = poseStamped.header.frame_id;
  marker.header.stamp = poseStamped.header.stamp;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = poseStamped.pose;
  marker.pose.position.z += 0.5;
  marker.scale.x = 4.0;
  marker.scale.y = 0.45;
  marker.scale.z = 0.45;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  go_to_waypoint_visualization_pub_->publish(marker);
}

void PlannerControlInterface::runHoming(bool exe_path) {
  planner_common::ServiceCall<planner_msgs::srv::PlannerHoming> plan_srv;
  plan_srv.request.header.stamp = planner_common::now();
  plan_srv.request.header.frame_id = world_frame_id_;
  trigger_mode_ = PlannerTriggerModeType::kAuto;
  if (planner_common::call(planner_homing_client_, plan_srv)) {
    runPlanner(exe_path);
  }
}

void PlannerControlInterface::runInitialization() {
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "Start initialization ...");
  planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  pci_manager_->initMotion();
}

void PlannerControlInterface::runSearch(bool exe_path) {
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Start searching ...");

  planner_common::ServiceCall<planner_msgs::srv::PlannerSetPlanningMode> planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::srv::PlannerSetPlanningMode::Request::MANUAL;
  planner_common::call(planner_set_trigger_mode_client_, planning_mode_srv);

  planner_common::ServiceCall<planner_msgs::srv::PlannerSearch> plan_srv;
  plan_srv.request.header.stamp = planner_common::now();
  plan_srv.request.header.frame_id = world_frame_id_;
  plan_srv.request.use_current_state = use_current_state_;
  plan_srv.request.bound_mode = bound_mode_;

  if (!use_current_state_) {
    plan_srv.request.source.position.x = source_setpoint_.position.x;
    plan_srv.request.source.position.y = source_setpoint_.position.y;
    plan_srv.request.source.position.z = source_setpoint_.position.z;
    plan_srv.request.source.orientation.x = source_setpoint_.orientation.x;
    plan_srv.request.source.orientation.y = source_setpoint_.orientation.y;
    plan_srv.request.source.orientation.z = source_setpoint_.orientation.z;
    plan_srv.request.source.orientation.w = source_setpoint_.orientation.w;
  } else {
    plan_srv.request.source.position.x = source_setpoint_.position.x;
    plan_srv.request.source.position.y = source_setpoint_.position.y;
    plan_srv.request.source.position.z = source_setpoint_.position.z;
    plan_srv.request.source.orientation.x = source_setpoint_.orientation.x;
    plan_srv.request.source.orientation.y = source_setpoint_.orientation.y;
    plan_srv.request.source.orientation.z = source_setpoint_.orientation.z;
    plan_srv.request.source.orientation.w = source_setpoint_.orientation.w;
  }
  plan_srv.request.target.position.x = target_setpoint_.position.x;
  plan_srv.request.target.position.y = target_setpoint_.position.y;
  plan_srv.request.target.position.z = target_setpoint_.position.z;
  plan_srv.request.target.orientation.x = target_setpoint_.orientation.x;
  plan_srv.request.target.orientation.y = target_setpoint_.orientation.y;
  plan_srv.request.target.orientation.z = target_setpoint_.orientation.z;
  plan_srv.request.target.orientation.w = target_setpoint_.orientation.w;

  if (planner_common::call(planner_search_client_, plan_srv)) {
    if (!plan_srv.response.path.empty()) {
      if (exe_path) {
        std::vector<geometry_msgs::msg::Pose> path_to_be_exe;
        pci_manager_->executePath(plan_srv.response.path, path_to_be_exe);
        current_path_ = path_to_be_exe;
      }
    } else {
      ROS_WARN_THROTTLE(1, "Planner Search returned an empty path");
    }
  } else {
    ROS_WARN_THROTTLE(1, "Planner Search service failed");
    planner_common::sleep(0.5);
  }
  planner_iteration_++;
}

geometry_msgs::msg::Pose PlannerControlInterface::getPoseToStart() {
  geometry_msgs::msg::Pose ret;
  // use current state as default
  ret.position.x = 0.0;
  ret.position.y = 0.0;
  ret.position.z = 0.0;
  ret.orientation.x = 0.0;
  ret.orientation.y = 0.0;
  ret.orientation.z = 0.0;
  ret.orientation.w = 1.0;

  // Use the last waypoint as a starting pose if required to plan ahead
  if (pci_manager_->planAhead() && (current_path_.size()))
    ret = current_path_.back();
  return ret;
}

bool PlannerControlInterface::loadParams() {
  std::string ns = planner_common::nodeName();
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Loading: %s", ns.c_str());

  // Required params for robot interface.
  if (!pci_manager_->loadParams(ns)) return false;

  // Other params.
  std::string param_name;
  std::string parse_str;
  param_name = ns + "/trigger_mode";
  planner_common::param::get(param_name, parse_str);
  if (!parse_str.compare("kManual"))
    trigger_mode_ = PlannerTriggerModeType::kManual;
  else if (!parse_str.compare("kAuto"))
    trigger_mode_ = PlannerTriggerModeType::kAuto;
  else {
    trigger_mode_ = PlannerTriggerModeType::kManual;
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "No trigger mode setting, set it to kManual.");
  }

  param_name = ns + "/world_frame_id";
  if (!planner_common::param::get(param_name, world_frame_id_)) {
    world_frame_id_ = "world";
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "No world_frame_id setting, set it to: %s.",
                  world_frame_id_.c_str());
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Done.");
  return true;
}

void PlannerControlInterface::odometryCallback(const nav_msgs::msg::Odometry& odo) {
  current_pose_.position.x = odo.pose.pose.position.x;
  current_pose_.position.y = odo.pose.pose.position.y;
  current_pose_.position.z = odo.pose.pose.position.z;
  current_pose_.orientation.x = odo.pose.pose.orientation.x;
  current_pose_.orientation.y = odo.pose.pose.orientation.y;
  current_pose_.orientation.z = odo.pose.pose.orientation.z;
  current_pose_.orientation.w = odo.pose.pose.orientation.w;
  pci_manager_->setState(current_pose_);
  pci_manager_->setCurrentVelocity(odo.twist.twist.linear);
  if (!pose_is_ready_) {
    previous_pose_ = current_pose_;
  } else {
    Eigen::Vector3d prev_state(previous_pose_.position.x,
                               previous_pose_.position.y,
                               previous_pose_.position.z);
    Eigen::Vector3d curr_state(current_pose_.position.x,
                               current_pose_.position.y,
                               current_pose_.position.z);
    const double kMinDist = 5.0;
    if ((prev_state - curr_state).norm() > kMinDist) {
      if (menu_initialized) {
        geometry_msgs::msg::Pose new_pose;
        new_pose.position = current_pose_.position;
        new_pose.orientation.x = 0.0;
        new_pose.orientation.y = 0.0;
        new_pose.orientation.z = 0.0;
        new_pose.orientation.w = 1.0;
        semantic_server->setPose("semantic", new_pose);
        semantic_server->applyChanges();
        previous_pose_ = current_pose_;
      }
    }
  }
  pose_is_ready_ = true;
}

void PlannerControlInterface::poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
  processPose(pose.pose.pose);
}

void PlannerControlInterface::poseStampedCallback(
    const geometry_msgs::msg::PoseStamped& pose) {
  processPose(pose.pose);
}

void PlannerControlInterface::processPose(const geometry_msgs::msg::Pose& pose) {
  current_pose_.position.x = pose.position.x;
  current_pose_.position.y = pose.position.y;
  current_pose_.position.z = pose.position.z;
  current_pose_.orientation.x = pose.orientation.x;
  current_pose_.orientation.y = pose.orientation.y;
  current_pose_.orientation.z = pose.orientation.z;
  current_pose_.orientation.w = pose.orientation.w;
  pci_manager_->setState(current_pose_);
  if (!pose_is_ready_) {
    previous_pose_ = current_pose_;
  } else {
    Eigen::Vector3d prev_state(previous_pose_.position.x,
                               previous_pose_.position.y,
                               previous_pose_.position.z);
    Eigen::Vector3d curr_state(current_pose_.position.x,
                               current_pose_.position.y,
                               current_pose_.position.z);
    const double kMinDist = 5.0;
    if ((prev_state - curr_state).norm() > kMinDist) {
      if (menu_initialized) {
        geometry_msgs::msg::Pose new_pose;
        new_pose.position = current_pose_.position;
        new_pose.orientation.x = 0.0;
        new_pose.orientation.y = 0.0;
        new_pose.orientation.z = 0.0;
        new_pose.orientation.w = 1.0;
        semantic_server->setPose("semantic", new_pose);
        semantic_server->applyChanges();
        previous_pose_ = current_pose_;
      }
    }
  }
  pose_is_ready_ = true;
}

void PlannerControlInterface::initIMarker() {
  {
    // create an interactive marker for our server
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = world_frame_id_;
    int_marker.header.stamp = planner_common::now();
    int_marker.name = source_marker_name;
    int_marker.description = "Source Pose";

    int_marker.pose.position.x = current_pose_.position.x;
    int_marker.pose.position.y = current_pose_.position.y;
    int_marker.pose.position.z = 1.5;

    // create a grey box marker
    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.8;
    box_marker.color.g = 0.1;
    box_marker.color.b = 0.1;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::msg::InteractiveMarkerControl control;

    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_xy";
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_z";
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    imarker_server_->insert(int_marker);
    imarker_server_->setCallback(
        int_marker.name,
        std::bind(&PlannerControlInterface::processFeedback, this, std::placeholders::_1));
  }
  {
    // create an interactive marker for our server
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = world_frame_id_;
    int_marker.header.stamp = planner_common::now();
    int_marker.name = target_marker_name;
    int_marker.description = "Target Pose";

    int_marker.pose.position.x = current_pose_.position.x + 1.0;
    int_marker.pose.position.y = current_pose_.position.y;
    int_marker.pose.position.z = 1.5;

    // create a grey box marker
    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.1;
    box_marker.color.g = 0.8;
    box_marker.color.b = 0.1;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::msg::InteractiveMarkerControl control;

    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_xy";
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "move_z";
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    imarker_server_->insert(int_marker);
    imarker_server_->setCallback(
        int_marker.name,
        std::bind(&PlannerControlInterface::processFeedback, this, std::placeholders::_1));
  }
  // 'commit' changes and send to all clients
  imarker_server_->applyChanges();
}

void PlannerControlInterface::processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  if (!feedback->marker_name.compare(source_marker_name)) {
    // update source wp.
    source_setpoint_.position.x = feedback->pose.position.x;
    source_setpoint_.position.y = feedback->pose.position.y;
    source_setpoint_.position.z = feedback->pose.position.z;
    source_setpoint_.orientation.x = feedback->pose.orientation.x;
    source_setpoint_.orientation.y = feedback->pose.orientation.y;
    source_setpoint_.orientation.z = feedback->pose.orientation.z;
    source_setpoint_.orientation.w = feedback->pose.orientation.w;
  } else if (!feedback->marker_name.compare(target_marker_name)) {
    // update target wp.
    target_setpoint_.position.x = feedback->pose.position.x;
    target_setpoint_.position.y = feedback->pose.position.y;
    target_setpoint_.position.z = feedback->pose.position.z;
    target_setpoint_.orientation.x = feedback->pose.orientation.x;
    target_setpoint_.orientation.y = feedback->pose.orientation.y;
    target_setpoint_.orientation.z = feedback->pose.orientation.z;
    target_setpoint_.orientation.w = feedback->pose.orientation.w;
  }
}

// Semantics
void PlannerControlInterface::initSemanticIMarker() {
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = world_frame_id_;
  int_marker.header.stamp = planner_common::now();
  int_marker.name = "semantic";
  int_marker.description = "";
  int_marker.pose.position.x = current_pose_.position.x + 1.5;
  int_marker.pose.position.y = current_pose_.position.y;
  int_marker.pose.position.z = current_pose_.position.z;
  int_marker.scale = control_size;

  visualization_msgs::msg::InteractiveMarkerControl x_control;
  x_control.name = "x_control";
  x_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(x_control);

  visualization_msgs::msg::InteractiveMarkerControl y_control;
  y_control.name = "y_control";
  y_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  y_control.orientation.x = 0;
  y_control.orientation.y = 0;
  y_control.orientation.z = 0.707;
  y_control.orientation.w = 0.707;
  int_marker.controls.push_back(y_control);

  visualization_msgs::msg::InteractiveMarkerControl z_control;
  z_control.name = "z_control";
  z_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  z_control.orientation.x = 0;
  z_control.orientation.y = 0.707;
  z_control.orientation.z = 0;
  z_control.orientation.w = 0.707;
  int_marker.controls.push_back(z_control);

  visualization_msgs::msg::Marker button_box_marker;
  button_box_marker.type = visualization_msgs::msg::Marker::CUBE;
  button_box_marker.scale.x = 1.0;
  button_box_marker.scale.y = 1.0;
  button_box_marker.scale.z = 1.0;
  button_box_marker.color.r = 0.5;
  button_box_marker.color.g = 0.5;
  button_box_marker.color.b = 0.5;
  button_box_marker.color.a = 0.65;

  visualization_msgs::msg::InteractiveMarkerControl button_control;
  button_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  button_control.name = "button_control";
  button_control.description = "menu_button";
  button_control.markers.push_back(button_box_marker);
  button_control.always_visible = true;

  int_marker.controls.push_back(button_control);

  semantic_server->insert(int_marker);
  semantic_server->setCallback(
      int_marker.name,
      std::bind(&PlannerControlInterface::semanticMarkerFeedback, this, std::placeholders::_1));

  if (!menu_initialized) {
    class_entry_handle = menu_handler.insert("Class");
    accept_entry_handle = menu_handler.insert(
        "Accept",
        std::bind(&PlannerControlInterface::acceptButtonFeedback, this, std::placeholders::_1));

    sub_class_entry_handle = menu_handler.insert(
        class_entry_handle, kStaircaseStr,
        std::bind(&PlannerControlInterface::selectSemanticsFeedback, this,
                    std::placeholders::_1));
    menu_handler.setCheckState(sub_class_entry_handle,
                               interactive_markers::MenuHandler::UNCHECKED);
    sub_class_entry_handle = menu_handler.insert(
        class_entry_handle, kDoorStr,
        std::bind(&PlannerControlInterface::selectSemanticsFeedback, this,
                    std::placeholders::_1));
    menu_handler.setCheckState(sub_class_entry_handle,
                               interactive_markers::MenuHandler::UNCHECKED);

    menu_initialized = true;
  }
  menu_handler.apply(*semantic_server, "semantic");

  semantic_server->applyChanges();
}

void PlannerControlInterface::semanticMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  visualization_msgs::msg::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  semantic_position.x = marker.pose.position.x;
  semantic_position.y = marker.pose.position.y;
  semantic_position.z = marker.pose.position.z;
}

void PlannerControlInterface::acceptButtonFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  visualization_msgs::msg::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  semantic_position.x = marker.pose.position.x;
  semantic_position.y = marker.pose.position.y;
  semantic_position.z = marker.pose.position.z;

  semantic_location.point = semantic_position;
  semantic_location.type.value = current_semantic_class_.value;

  semantic_pub->publish(semantic_location);
}

void PlannerControlInterface::selectSemanticsFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  visualization_msgs::msg::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  menu_handler.setCheckState(sub_class_entry_handle,
                             interactive_markers::MenuHandler::UNCHECKED);
  sub_class_entry_handle = feedback->menu_entry_id;
  menu_handler.setCheckState(sub_class_entry_handle,
                             interactive_markers::MenuHandler::CHECKED);
  menu_handler.reApply(*semantic_server);
  semantic_server->applyChanges();
  std::string semantic_class;
  menu_handler.getTitle(sub_class_entry_handle, semantic_class);
  if (!semantic_class.compare(kStaircaseStr))
    current_semantic_class_.value =
        planner_semantic_msgs::msg::SemanticClass::STAIRCASE;
  else if (!semantic_class.compare(kDoorStr))
    current_semantic_class_.value = planner_semantic_msgs::msg::SemanticClass::DOOR;
  else
    current_semantic_class_.value = planner_semantic_msgs::msg::SemanticClass::NONE;
}

}  // namespace explorer
