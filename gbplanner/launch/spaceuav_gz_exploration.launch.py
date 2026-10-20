"""gbplanner exploration for one ArduCopter drone of the SpaceUav Gazebo stack.

Start the simulator first, with the 3D lidar and without the TUIs (they also
publish <drone>/map -> <drone>/base_link):

    ~/AFlightForward/spaceuav-main-flight-stack/scripts/run_gazebo_mixed_sitl.sh \
        --lidar3d --no-tui

then:

    ros2 launch gbplanner spaceuav_gz_exploration.launch.py drone:=SpaceUavDrone1

and use the GbPlanner panel in RViz: "Initialization" (GUIDED, arm, takeoff,
turn in place) then "Start Planner".

Everything runs in the /<drone> namespace. Frames: <drone>/map (mavros local
ENU frame, origin at the spawn point) -> <drone>/base_link -> <drone>/lidar_3d.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Mount of the gpu_lidar that run_gazebo_mixed_sitl.sh --lidar3d adds to the
# Iris (x y z in metres, relative to base_link).
LIDAR_3D_MOUNT = ('0.0', '0.0', '0.2')


def launch_setup(context):
    drone = LaunchConfiguration('drone').perform(context)
    config_folder = LaunchConfiguration('config_folder').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    gbplanner_share = get_package_share_directory('gbplanner')
    config_dir = os.path.join(gbplanner_share, 'config', config_folder)
    pci_config = LaunchConfiguration('pci_config').perform(context) or os.path.join(
        get_package_share_directory('pci_mavros'), 'config', 'pci_mavros_sim.yaml')

    map_frame = f'{drone}/map'
    base_frame = f'{drone}/base_link'
    lidar_frame = f'{drone}/lidar_3d'

    gbplanner = Node(
        package='gbplanner', executable='gbplanner_node', name='gbplanner_node',
        namespace=drone, output='screen',
        parameters=[
            os.path.join(config_dir, 'gbplanner_node.yaml'),
            {
                'use_sim_time': use_sim_time,
                'behavior_tree_path': os.path.join(config_dir, 'main_tree.xml'),
                'PlanningParams.global_frame_id': map_frame,
                'world_frame': map_frame,
            },
        ],
        remappings=[
            (f'/{drone}/gbplanner_node/pointcloud', LaunchConfiguration('pointcloud_topic')),
        ],
    )

    pci = Node(
        package='pci_mavros', executable='pci_mavros_node', name='pci_mavros_node',
        namespace=drone, output='screen',
        parameters=[
            pci_config,
            {
                'use_sim_time': use_sim_time,
                'world_frame_id': map_frame,
                'body_frame_id': base_frame,
                'publish_tf': LaunchConfiguration('publish_tf').perform(context).lower() == 'true',
            },
        ],
        remappings=[
            ('planner_server', 'gbplanner_ros'),
            ('planner_homing_server', 'gbplanner_ros_homing'),
        ],
    )

    lidar_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lidar_3d_tf', namespace=drone,
        arguments=['--x', LIDAR_3D_MOUNT[0], '--y', LIDAR_3D_MOUNT[1], '--z', LIDAR_3D_MOUNT[2],
                   '--frame-id', base_frame, '--child-frame-id', lidar_frame],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='gbplanner_rviz', namespace=drone,
        arguments=['-d', os.path.join(get_package_share_directory('gbplanner_ui'),
                                      'config', 'gbplanner.rviz'),
                   '-f', map_frame],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [gbplanner, pci, lidar_tf, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone', default_value='SpaceUavDrone1',
                              description='Drone name from gazebo_fleet.conf (ArduPilot drone)'),
        DeclareLaunchArgument('config_folder', default_value='uav/spaceuav_gz',
                              description='Folder under gbplanner/config with '
                                          'gbplanner_node.yaml and main_tree.xml'),
        DeclareLaunchArgument('pci_config', default_value='',
                              description='pci_mavros parameter file (default: '
                                          'pci_mavros/config/pci_mavros_sim.yaml)'),
        DeclareLaunchArgument('pointcloud_topic',
                              default_value=[ '/', LaunchConfiguration('drone'), '/lidar_3d/points'],
                              description='PointCloud2 from the 3D lidar'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='pci_mavros publishes <drone>/map -> <drone>/base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
