"""Nodes under test for tools/tag_sweep.py: robot_state_publisher, apriltag_ros on camera_left, dock_tag_node.
    ros2 launch tools/sweep.launch.py decimate:=2.0        (from luna_ws/src/luna_robot, workspace sourced)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory('luna_robot')
    with open(os.path.join(share, 'urdf', 'luna.urdf')) as f:
        urdf = f.read()
    decimate = float(context.perform_substitution(LaunchConfiguration('decimate')))
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': urdf}]),
        Node(package='apriltag_ros', executable='apriltag_node', name='apriltag', namespace='camera_left',
             parameters=[os.path.join(share, 'config', 'apriltag.yaml'),
                         {'tag.frames': ['dock_tag_left'], 'detector.decimate': decimate}],
             remappings=[('image_rect', 'image_raw')]),
        Node(package='luna_robot', executable='dock_tag_node', name='luna_dock_tag',
             parameters=[os.path.join(share, 'config', 'luna.yaml'), {'cameras': ['left']}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('decimate', default_value='2.0'), OpaqueFunction(function=_nodes)])
