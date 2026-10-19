"""Nav2 for Luna (Humble).  ros2 launch luna_robot nav2.launch.py [obstacle_sensors:=all|lidar]
Needs /map + map→odom (slam.launch.py), /scan, and odom→base_footprint (base_node).
Every velocity Nav2 makes ends on /cmd_vel, i.e. base_node's mux and safety gate (config/nav2.yaml).
obstacle_sensors:=all (default) loads config/nav2_obstacle_sensors.yaml on top: the depth camera (/oak/points) and
the corner ToF (/luna/tof/*) join the costmaps and collision_monitor. lidar: /scan only, as before. Same nodes
either way; only the parameters differ."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from luna_robot.launch_util import IGNORE_LATE_SIGINT

SERVERS = {                       # node → package; started and kept alive by the lifecycle manager
    'controller_server': 'nav2_controller',
    'planner_server': 'nav2_planner',
    'behavior_server': 'nav2_behaviors',
    'bt_navigator': 'nav2_bt_navigator',
    'velocity_smoother': 'nav2_velocity_smoother',
    'collision_monitor': 'nav2_collision_monitor',
}
# controller/behaviors → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → /cmd_vel
# (collision_monitor's topics are parameters in config/nav2.yaml)
REMAPS = {
    'controller_server': [('cmd_vel', 'cmd_vel_nav')],
    'behavior_server': [('cmd_vel', 'cmd_vel_nav')],
    'velocity_smoother': [('cmd_vel', 'cmd_vel_nav')],
}


OBSTACLE_SENSORS = {'lidar': [], 'all': ['nav2_obstacle_sensors.yaml']}    # overlays on params_file
BT_XML = 'navigate_to_pose_keep_near_obstacles.xml'


def servers(context, share):
    sensors = context.perform_substitution(LaunchConfiguration('obstacle_sensors'))
    if sensors not in OBSTACLE_SENSORS:
        raise ValueError(f'obstacle_sensors:={sensors}: expected {" or ".join(OBSTACLE_SENSORS)}')
    params = [LaunchConfiguration('params_file')] + [os.path.join(share, 'config', f) for f in OBSTACLE_SENSORS[sensors]]
    # Recoveries keep the costmaps near the robot: low obstacles only the camera / ToF saw are not forgotten
    params.append({'default_nav_to_pose_bt_xml': os.path.join(share, 'behavior_trees', BT_XML)})
    return [Node(package=pkg, executable=name, name=name, parameters=params, remappings=REMAPS.get(name, []),
                 prefix=IGNORE_LATE_SIGINT, exec_name=name)
            for name, pkg in SERVERS.items()]


def generate_launch_description():
    share = get_package_share_directory('luna_robot')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2.yaml')),
        DeclareLaunchArgument('obstacle_sensors', default_value='all', description='all | lidar'),
        OpaqueFunction(function=servers, args=[share]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             parameters=[{'autostart': True, 'node_names': list(SERVERS), 'bond_timeout': 4.0}],
             prefix=IGNORE_LATE_SIGINT, exec_name='lifecycle_manager_navigation'),
    ])
