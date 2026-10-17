"""Nav2 for Luna (Humble).  ros2 launch luna_robot nav2.launch.py
Needs /map + map→odom (slam.launch.py), /scan, and odom→base_footprint (base_node).
Every velocity Nav2 makes ends on /cmd_vel, i.e. base_node's mux and safety gate (config/nav2.yaml)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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


def generate_launch_description():
    share = get_package_share_directory('luna_robot')
    params = LaunchConfiguration('params_file')
    nodes = [Node(package=pkg, executable=name, name=name, parameters=[params], remappings=REMAPS.get(name, []),
                  prefix=IGNORE_LATE_SIGINT, exec_name=name)
             for name, pkg in SERVERS.items()]
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2.yaml')),
        *nodes,
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             parameters=[{'autostart': True, 'node_names': list(SERVERS), 'bond_timeout': 4.0}],
             prefix=IGNORE_LATE_SIGINT, exec_name='lifecycle_manager_navigation'),
    ])
