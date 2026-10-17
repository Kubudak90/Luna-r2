"""Luna bringup.  ros2 launch luna_robot luna.launch.py            (mock MCU, no hardware)
               ros2 launch luna_robot luna.launch.py mock:=false  (Teensy over micro-ROS + RPLIDAR C1)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Starts a C++ node with SIGINT ignored. rclcpp still installs its own handler, and puts SIG_IGN back when it shuts
# down, so a second SIGINT during teardown can't kill the node with -2. launch sends that second SIGINT when stdin
# is not a terminal (docker without -t, scripts, services) after the process group already got one.
IGNORE_LATE_SIGINT = """sh -c 'trap "" INT; exec "$0" "$@"'"""


def generate_launch_description():
    share = get_package_share_directory('luna_robot')
    params = os.path.join(share, 'config', 'luna.yaml')
    with open(os.path.join(share, 'urdf', 'luna.urdf')) as f:
        urdf = f.read()
    mock = LaunchConfiguration('mock')
    lidar = LaunchConfiguration('lidar')
    actions = [
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': urdf}],
             prefix=IGNORE_LATE_SIGINT, exec_name='robot_state_publisher'),
        Node(package='luna_robot', executable='base_node', name='luna_base', parameters=[params]),
        Node(package='luna_robot', executable='head_node', name='luna_head', parameters=[params]),
        Node(package='luna_robot', executable='dock_node', name='luna_dock', parameters=[params]),
        Node(package='luna_robot', executable='face_bridge_node', name='luna_face_bridge', parameters=[params]),
        Node(package='luna_robot', executable='mock_mcu_node', name='luna_mock_mcu', parameters=[params], condition=IfCondition(mock)),
        Node(package='micro_ros_agent', executable='micro_ros_agent', arguments=['serial', '--dev', LaunchConfiguration('mcu_port')],
             condition=UnlessCondition(mock)),
    ]
    try:  # Slamtec driver is optional so the mock stack runs without it
        sllidar = get_package_share_directory('sllidar_ros2')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sllidar, 'launch', 'sllidar_c1_launch.py')),
            launch_arguments={'serial_port': LaunchConfiguration('lidar_port'), 'frame_id': 'laser'}.items(),
            condition=IfCondition(lidar)))
    except Exception:
        pass
    return LaunchDescription(actions)
