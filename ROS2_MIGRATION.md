# ROS 2 (Jazzy) Migration

Target: ROS 2 Jazzy, Ubuntu 24.04, Gazebo Harmonic, `ament_cmake` + `colcon`.
Reference: https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html

## Status

| Package | State |
|---|---|
| `planner_msgs`, `planner_semantic_msgs` | ported, builds |
| `kdtree`, `planner_common` | ported, builds |
| `map_manager` | ported, builds (voxblox TSDF server embedded in the planner node) |
| `gbplanner` | ported, builds, runs: plans and returns paths (see Verification) |
| `planner_control_interface` | ported, builds, runs |
| `pci_mavros` | **new**, replaces `pci_general` for ArduCopter + mavros |
| `gbplanner_ui` | ported to RViz2 (`rviz_common::Panel`), builds |
| `adaptive_obb` | ported on `lateforwork04/adaptive_obb_ros@ros2-jazzy` |
| `ugv_simulator`, `planner_gazebo_sim`, ROS 1 launch files | removed |
| `pci_general`, `manhole_detector_ros`, RotorS / SMB / subt simulators | not ported (replaced by the SpaceUav Gazebo Harmonic stack) |

### Verification so far
- Whole workspace builds with `colcon` on Jazzy.
- Launch smoke test with a fake mavros odometry + synthetic 32x360 lidar cloud
  (no simulator): voxblox integrates the cloud, gbplanner builds a graph
  (128 vertices / 366 edges), returns a path, `pci_mavros` streams setpoints on
  `mavros/setpoint_raw/local` and reports ready for the next iteration.
- **Not yet run** against the real Gazebo + ArduCopter SITL stack (arming,
  takeoff, setpoint tracking in GUIDED are untested).

## What changed (and why)

- **Interfaces**: rosidl requires PascalCase interface names and UPPER_CASE
  constants: `planner_srv` -> `PlannerSrv`, `kExtendedBound` -> `EXTENDED_BOUND`,
  `pathFollowerAction` -> `PathFollower`. Every external consumer must follow.
- **`planner_common/ros2_compat.h`**: keeps the ~500 ROS 1 logging calls and
  ~160 global `ros::param::get` calls working on rclcpp (`ROS_*` macros ->
  `RCLCPP_*`; `"<node>/Group/name"` -> parameter `Group.name`; nodes are created
  with `automatically_declare_parameters_from_overrides`). Also `ServiceCall<T>`
  / `call()` (blocking call with timeout), `advertiseService()`, `subscribe()`.
- **Threading**: nodes spin in a `MultiThreadedExecutor`; planner callbacks stay
  in the default mutually exclusive group (ROS 1 single-thread semantics) and
  service clients called from callbacks use a separate group, so the blocking
  planner <-> PCI calls cannot deadlock. The PCI loop is a 20 Hz timer.
- **tf -> tf2**, `ros::package` -> `ament_index_cpp`, `ros::serialization` ->
  `rclcpp::Serialization` (graph files saved by the ROS 1 version cannot be loaded).
- **Parameters**: YAML is now `/**/<node>: ros__parameters:`. rosparam's
  `rad()`/`deg()` are evaluated at conversion time. Types must match what the code
  reads (`50.0`, not `50`, for voxblox doubles).
- **Frames**: visualization and homing use `PlanningParams.global_frame_id`
  (per-drone `<drone>/map`) instead of a hard-coded `world`.
- **RViz**: new RViz2 layout, relative topics; the panel follows RViz' namespace.

## Dependencies

| Dependency | Resolution |
|---|---|
| voxblox | `GabrieleSantangelo/voxblox-ros2` (Jazzy CI, `EsdfServer/TsdfServer(rclcpp::Node*)`). The cmrobotics `ros2` branch was only a partial port (no `EsdfServer`, publishing disabled). NTNU's fork changes (fast-integrator abort criterion, ESDF colour) are not carried over; gbplanner does not need them. Build with `-DCMAKE_POSITION_INDEPENDENT_CODE=ON`. |
| catkin wrappers (`glog_catkin`, `protobuf_catkin`, `minkindr`, ...) | system packages / voxblox-ros2's own ament packages |
| `catkin_simple` | plain `ament_cmake` |
| BehaviorTree.CPP | `ros-jazzy-behaviortree-cpp` (v4; trees were already `BTCPP_format="4"`) |
| grid_map, pcl_ros | `ros-jazzy-grid-map`, `ros-jazzy-pcl-ros` |
| RotorS / gz Garden / patched gz-sim | replaced by the SpaceUav Gazebo Harmonic stack + `pci_mavros` |

## Known issues / follow-ups

- voxblox-ros2 `tsdf_server.cc`: `min_time_between_msgs_.from_seconds(...)` is a
  static call whose result is discarded, so `min_time_between_msgs_sec` is ignored
  and clouds are integrated at most once per second (fix upstream or in a fork).
- glog's failure signal handler (kept from ROS 1) prints a stack trace on SIGTERM
  at shutdown; harmless.
- The sensor must observe the space around the robot for the first graph to
  connect: the 3D lidar is 32 beams, +/-45 deg (a +/-15 deg lidar leaves the
  root box unknown and planning fails).
- PX4 drones are not supported by `pci_mavros` (ArduPilot GUIDED only).
