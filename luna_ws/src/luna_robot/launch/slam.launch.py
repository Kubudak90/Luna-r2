"""slam_toolbox for the RPLIDAR C1.
  ros2 launch luna_robot slam.launch.py                                   build a map (online, async)
  ros2 launch luna_robot slam.launch.py slam_mode:=localization map:=/root/.luna/maps/home   (+ slam_watchdog_node)
Save a map while mapping (both files go next to each other; the dock pose file joins them later):
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /root/.luna/maps/home}"
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: /root/.luna/maps/home}}"   (.pgm/.yaml)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from luna_robot.launch_util import IGNORE_LATE_SIGINT


def slam_node(context):
    share = get_package_share_directory('luna_robot')
    mode = context.launch_configurations['slam_mode']
    params = [os.path.join(share, 'config', 'slam_mapping.yaml')]
    if mode == 'localization':
        params += [os.path.join(share, 'config', 'slam_localization.yaml'),
                   {'map_file_name': os.path.expanduser(context.launch_configurations['map'])}]
        executable = 'localization_slam_toolbox_node'
    elif mode == 'mapping':
        executable = 'async_slam_toolbox_node'
    else:
        raise ValueError(f"slam_mode must be mapping or localization, not '{mode}'")
    localization = mode == 'localization'
    # Localization: slam_toolbox sometimes locks up for good; slam_watchdog_node kills it, the launch respawns it with
    # the same saved map, and the watchdog hands it the robot's pose (core/slam_watchdog.py). Mapping is not
    # restarted: the map being built would be lost.
    nodes = [Node(package='slam_toolbox', executable=executable, name='slam_toolbox', parameters=params,
                  prefix=IGNORE_LATE_SIGINT, exec_name='slam_toolbox', respawn=localization, respawn_delay=1.0)]
    if localization:
        nodes.append(Node(package='luna_robot', executable='slam_watchdog_node', name='luna_slam_watchdog'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam_mode', default_value='mapping', description='mapping | localization'),
        DeclareLaunchArgument('map', default_value='~/.luna/maps/home',
                              description='saved pose graph, path without .posegraph/.data (localization)'),
        OpaqueFunction(function=slam_node),
    ])
