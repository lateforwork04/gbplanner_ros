# ROS 2 (Jazzy) Migration Plan

Target: ROS 2 Jazzy on Ubuntu 24.04, Gazebo Harmonic, `ament_cmake` + `colcon`.
Reference: https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html

## Package migration order (bottom-up)

| # | Package | Main work |
|---|---------|-----------|
| 1 | `kdtree` | Pure C; swap `catkin_simple` -> plain `ament_cmake` |
| 2 | `planner_msgs`, `planner_semantic_msgs` | `rosidl_generate_interfaces`; **rename all interfaces to PascalCase** (see below); actionlib_msgs -> `action_msgs` |
| 3 | `planner_common` | `tf` -> `tf2_ros`; `ros::serialization` -> `rclcpp::Serialization` (graph_manager.cpp save/load) |
| 4 | `map_manager` | Blocked on voxblox (see dependency risks) |
| 5 | `gbplanner` | rclcpp node, params via `declare_parameter`, `roslib` -> `ament_index_cpp`, timers/services/pubs |
| 6 | `planner_control_interface` | actionlib client -> `rclcpp_action`; interactive_markers ROS 2 API |
| 7 | `gbplanner_ui` | rviz Panel -> `rviz_common::Panel`, package.xml format 1 -> 3 |
| 8 | `planner_gazebo_sim`, `ugv_simulator/ugv_gazebo` | Gazebo Harmonic + `ros_gz`; drop `dynamic_reconfigure` |
| 9 | launch / config | 12 `.launch` files -> ROS 2 launch; YAML params need `node_name: ros__parameters:` nesting |

## Mechanical changes that apply everywhere

- `catkin` / `catkin_simple` -> `ament_cmake` (`ament_cmake_auto` is the closest `catkin_simple` analogue)
- `package.xml` -> format 3, `message_generation`/`message_runtime` -> `rosidl_default_generators`/`rosidl_default_runtime` + `<member_of_group>rosidl_interface_packages</member_of_group>`
- `#include <foo/Bar.h>` -> `#include <foo/msg/bar.hpp>`, `foo::Bar` -> `foo::msg::Bar`, `ConstPtr` -> `SharedPtr`
- `ros::NodeHandle` -> `rclcpp::Node`; `ros::Time`/`ros::Duration` -> `rclcpp::Time`/`rclcpp::Duration`; `ROS_INFO` -> `RCLCPP_INFO(get_logger(), ...)`
- `tf` (tf1) -> `tf2_ros` + `tf2_eigen` / `tf2_geometry_msgs`
- `std_srvs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `sensor_msgs`, `cv_bridge`, `pcl_ros`, `pcl_conversions`, `interactive_markers` all exist in Jazzy

### Breaking: interface names

ROS 2 `rosidl` requires interface names to match `^[A-Z][A-Za-z0-9]*$`. These 26 files will fail to generate and must be renamed (e.g. `planner_srv.srv` -> `PlannerSrv.srv`, `pathFollowerAction.action` -> `PathFollower.action`). Every consumer, including external repos (`pci_general`, `adaptive_obb_ros`, ...), must be updated in step:

`pathFollowerAction.action`, `planner_string_trigger`, `planner_geofence`, `planner_go_to_waypoint`, `planner_search`, `planner_set_global_bound`, `planner_opening_approval`, `planner_set_search_mode`, `planner_srv`, `planner_request_path`, `planner_global`, `planner_homing`, `pci_initialization`, `pci_set_homing_pos`, `planner_dynamic_global_bound`, `planner_set_exp_mode`, `pci_to_waypoint`, `pci_geofence`, `pci_global`, `planner_set_planning_mode`, `pci_trigger`, `planner_set_homing_pos`, `pci_stop`, `pci_search`, `pci_homing_trigger`, `planner_set_vel`

## Dependency risk assessment

### High risk: no ROS 2 version, must port or replace

| Dependency | Used by | Issue | Suggested path |
|---|---|---|---|
| `voxblox` / `voxblox_ros` (ntnu-arl fork, `feature/esdf_colorization`) | `map_manager` | ROS 1 only, upstream unmaintained, no official ROS 2 port | Port voxblox core + write a thin rclcpp wrapper (community ports exist as a starting point), or switch mapping backend (nvblox, wavemap, OctoMap) |
| `protobuf_catkin`, `glog_catkin`, `gflags_catkin`, `eigen_catkin`, `eigen_checks`, `minkindr`, `minkindr_ros`, `numpy_eigen`, `catkin_boost_python_buildtool` | voxblox | catkin-wrapper packages, no ROS 2 builds | Drop; use system `libprotobuf-dev`, `libgoogle-glog-dev`, `libgflags-dev`, `libeigen3-dev`. minkindr core is header-only |
| `catkin_simple` | kdtree, gbplanner, planner_control_interface, planner_gazebo_sim | catkin-only | `ament_cmake_auto` |
| `adaptive_obb_ros` | gbplanner | ntnu-arl ROS 1 repo | Port (or move to a pure-C++ core lib) |
| `pci_general`, `manhole_detector_ros`, `gbplanner3` | runtime stack | ntnu-arl ROS 1 repos; also consume the renamed interfaces | Port alongside this repo |
| `rotors_simulator` (`dev/rmf_obelix`) | sim | ROS 1 + Gazebo Classic (EOL Jan 2025) | Replace with Gazebo Harmonic models + `ros_gz` |
| `ros_gz` fork (`garden_noetic`) + patched `gz-sim` (`fix/position_control`, `dev/multicopter_control`) | sim | Garden is EOL; Jazzy pairs with Harmonic. Custom patches must be rebased | Rebase patches onto gz-sim 8 (Harmonic), use upstream `ros_gz` jazzy |
| `smb_simulator` (`dev/noetic`), `subt_cave_sim`, `arl_gazebo_sim_ros`, `lidar_simulator` | sim | ROS 1 | Port, or keep only world/model assets |
| `multi_dof_joint_trajectory_rviz_plugins` | rviz | rviz1 plugin | Port to rviz2 |

### Medium risk: available, but API changes

| Dependency | Issue |
|---|---|
| `BehaviorTree.CPP` (pinned `catkin_support` branch) | XML is already `BTCPP_format="4"`, so use Jazzy's `ros-jazzy-behaviortree-cpp` (v4). Verify API compatibility of the pinned commit vs released v4 |
| `mav_comm` / `mav_msgs` | No official Jazzy release; community ROS 2 ports exist or vendor the needed messages |
| `grid_map_*` | Released for ROS 2 but check Jazzy binary availability (`ros-jazzy-grid-map`); may need source build |
| `actionlib` / `actionlib_msgs` | -> `rclcpp_action`; different client/server API |
| `roscpp_serialization` | -> `rclcpp::Serialization<T>`; changes on-disk format of saved graphs (old files unreadable) |
| `roslib` | -> `ament_index_cpp::get_package_share_directory` |
| `dynamic_reconfigure` (ugv_gazebo) | -> parameter callbacks |
| `gbplanner_ui` rviz panel | -> `rviz_common`, pluginlib export changes |

### Low risk: drop-in Jazzy packages

`std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`, `std_srvs`, `cv_bridge`, `pcl_ros`, `pcl_conversions`, `tf2_ros`, `interactive_markers`, `joy`, `twist_mux`, `interactive_marker_twist_server`, `octomap_msgs`, `octomap_ros`, Qt5, PCL, OpenCV, Eigen.

## Suggested strategy

1. Port the interface packages first (with renames) and run `ros1_bridge`-free ROS 2 builds of `kdtree` + msgs.
2. Decide the voxblox question early: it is the critical path for `map_manager` and therefore `gbplanner`.
3. Port `planner_common` -> `map_manager` -> `gbplanner` as composable nodes.
4. Do the simulation stack (Harmonic + ros_gz) in parallel, since it is mostly independent.
5. Update `vcstool/*.repos` to point at ROS 2 branches as each external repo is ported.
