#  <div align="center">**OmniPlanner: Universal Exploration and Inspection Path Planning Across Robot Morphologies (aka GBPlanner 3.0)**</div>

<div align="center"> <a href="https://ntnu-arl.github.io/omniplanner/"><img src="https://img.shields.io/badge/Homepage-1E88E5?style=flat-square" alt="Webpage"></a>  <a href="https://arxiv.org/abs/2603.04284"><img src="https://img.shields.io/badge/arXiv-78909C?style=flat-square" alt="arXiv"></a> <a href="https://www.youtube.com/watch?v=kT4hGuejhuQ"><img src="https://img.shields.io/badge/YouTube-E57373?style=flat-square" alt="YouTube"></a> </div>

<br>

> **OmniPlanner builds upon GBPlanner 2.0**, the exploration planning method that guided all robots of **Team CERBERUS** during the winning run at the **DARPA Subterranean Challenge**. An updated version of **GBPlanner 2.0** is available in the [`gbplanner2`](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2).

<br>

**OmniPlanner** is a unified graph-based planning framework that enables autonomous robots to explore unknown environments, inspect structures and regions of interest, and navigate to specified targets. Its modular formulation adapts the planning process to the motion and sensing characteristics of aerial, ground, and underwater platforms, allowing the same framework to generate feasible and informative paths across diverse robot morphologies and operating environments.

![swag](img/omniplanner_intro.png)
<sub><em>**OmniPlanner:** A unified framework for autonomous exploration, inspection, and target-reach planning with aerial, ground, and underwater robots.</em></sub>


For an extensive documentation, installation instructions, and demos please visit the documentation page of the repository here: [**Documetation**](https://github.com/ntnu-arl/gbplanner3_wiki/wiki).


## ROS 2 Jazzy port (`ros2-jazzy-migration` branch)

This branch runs on **ROS 2 Jazzy / Ubuntu 24.04 / Gazebo Harmonic**. The ROS 1 (Noetic)
instructions are on the upstream [`gbplanner3`](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner3)
branch. Migration notes, dependency status and what changed: [ROS2_MIGRATION.md](ROS2_MIGRATION.md).

### Dependencies
```bash
sudo apt install ros-jazzy-pcl-ros ros-jazzy-grid-map ros-jazzy-behaviortree-cpp \
  ros-jazzy-backward-ros ros-jazzy-mavros ros-jazzy-ros-gz libgoogle-glog-dev \
  libgflags-dev libprotobuf-dev protobuf-compiler python3-vcstool
```

### Workspace
```bash
mkdir -p ~/gbplanner_ws/src && cd ~/gbplanner_ws
vcs import < <(curl -sL https://raw.githubusercontent.com/lateforwork04/gbplanner_ros/ros2-jazzy-migration/vcstool/ros2_jazzy.repos)
touch src/mapping/voxblox/voxblox_skeleton/COLCON_IGNORE src/mapping/voxblox/voxblox_rviz_plugin/COLCON_IGNORE
source /opt/ros/jazzy/setup.bash
# voxblox builds static libraries that are linked into shared ones: needs -fPIC
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
source install/setup.bash
```

| Package | Contents |
|---|---|
| `planner_msgs`, `planner_semantic_msgs` | interfaces (PascalCase names, UPPER_CASE constants) |
| `kdtree`, `planner_common` | graph, params, `ros2_compat.h` (logging / params / clock / service helpers) |
| `map_manager` | voxblox TSDF map running inside the planner node |
| `gbplanner` | planner node, behavior trees, configs, launch files |
| `planner_control_interface` | PCI library (planner loop, services, interactive markers) |
| `pci_mavros` | PCI for ArduCopter (GUIDED) through mavros |
| `gbplanner_ui` | RViz2 control panel |

### Simulation: SpaceUav Gazebo Harmonic stack (ArduCopter + mavros)
```bash
# 1. simulator, with the 3D lidar on the ArduPilot drones and without the TUIs
#    (they also publish <drone>/map -> <drone>/base_link)
~/AFlightForward/spaceuav-main-flight-stack/scripts/run_gazebo_mixed_sitl.sh --lidar3d --no-tui
# 2. planner + control interface + RViz2 for one drone
ros2 launch gbplanner spaceuav_gz_exploration.launch.py drone:=SpaceUavDrone1
```
In the **GbPlanner Control** panel: *Initialization* (GUIDED, arm, take off, turn in place),
then *Start Planner*. *2D Goal Pose* + *Plan to Waypoint* sends the drone to a goal.

## Citation

```bibtex
@article{zacharia2026omniplanner,
  title   = {OmniPlanner: Universal Exploration and Inspection Path Planning across Robot Morphologies},
  author  = {Zacharia, Angelos and Dharmadhikari, Mihir and Singh, Mohit and Alexis, Kostas},
  journal = {arXiv preprint arXiv:2603.04284},
  year    = {2026},
  url     = {https://arxiv.org/abs/2603.04284}
}
```

## GBPlanner Legacy

![swag](img/cerberus_subt_winners.png)

Earlier versions of GBPlanner have been deployed on multiple aerial and ground robot platforms:

![robots](img/gbplanner3_robots.png)

For background on the methods and deployments that preceded OmniPlanner, please refer to the following publications:

**Graph-based subterranean exploration path planning using aerial and legged robots**
```bibtex
@article{dang2020graph,
  title={Graph-based subterranean exploration path planning using aerial and legged robots},
  author={Dang, Tung and Tranzatto, Marco and Khattak, Shehryar and Mascarich, Frank and Alexis, Kostas and Hutter, Marco},
  journal={Journal of Field Robotics},
  volume = {37},
  number = {8},
  pages = {1363-1388},  
  year={2020},
  note={Wiley Online Library}
}
```
**Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots**
```bibtex
@INPROCEEDINGS{9812401,
  author={Kulkarni, Mihir and Dharmadhikari, Mihir and Tranzatto, Marco and Zimmermann, Samuel and Reijgwart, Victor and De Petris, Paolo and Nguyen, Huan and Khedekar, Nikhil and Papachristos, Christos and Ott, Lionel and Siegwart, Roland and Hutter, Marco and Alexis, Kostas},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)}, 
  title={Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots}, 
  year={2022},
  volume={},
  number={},
  pages={3306-3313},
  doi={10.1109/ICRA46639.2022.9812401}}
```

## Acknowledgements

This work was supported in part by the Research Council of Norway through the NCEI project (Grant No. 357451), and by the European Commission under the Horizon Europe Programme through the SYNERGISE (Grant No. 101121321), AUTOASSESS (Grant No. 101120732), SPEAR (Grant No. 101119774), and DIGIFOREST (Grant No. 101070405) projects. The authors are solely responsible for the content and ideas presented here.

OmniPlanner is intended for civilian use only and is provided under the terms of the repository's [LICENSE](https://github.com/ntnu-arl/gbplanner_ros/blob/gbplanner3/LICENSE).

## Contact

For questions, please contact:

- [Angelos Zacharia](mailto:angelos.zacharia@ntnu.no)
- [Mihir Dharmadhikari](mailto:mihir.dharmadhikari@ntnu.no)
- [Mohit Singh](mailto:mohit.singh@ntnu.no)
- [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
