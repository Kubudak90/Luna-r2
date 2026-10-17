"""Luna bringup.  ros2 launch luna_robot luna.launch.py            (mock MCU, no hardware)
               ros2 launch luna_robot luna.launch.py sim:=true    (2D room simulator with LiDAR, for SLAM/Nav2)
               ros2 launch luna_robot luna.launch.py mock:=false  (Teensy over micro-ROS + RPLIDAR C1)
Mapping and navigation on top of this: luna_nav.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from luna_robot.launch_util import IGNORE_LATE_SIGINT


def generate_launch_description():
    share = get_package_share_directory('luna_robot')
    params = os.path.join(share, 'config', 'luna.yaml')
    with open(os.path.join(share, 'urdf', 'luna.urdf')) as f:
        urdf = f.read()
    mock = LaunchConfiguration('mock')
    sim = LaunchConfiguration('sim')
    lidar = LaunchConfiguration('lidar')
    actions = [
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('sim', default_value='false', description='2D simulator instead of the mock MCU'),
        DeclareLaunchArgument('sim_soc', default_value='0.6', description='simulator start state of charge 0..1'),
        DeclareLaunchArgument('auto_dock', default_value='true',
                              description='dock_node docks by itself on low battery (false when dock_nav_node decides)'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': urdf}],
             prefix=IGNORE_LATE_SIGINT, exec_name='robot_state_publisher'),
        Node(package='luna_robot', executable='base_node', name='luna_base', parameters=[params]),
        Node(package='luna_robot', executable='head_node', name='luna_head', parameters=[params]),
        Node(package='luna_robot', executable='dock_node', name='luna_dock',
             parameters=[params, {'auto_dock': ParameterValue(LaunchConfiguration('auto_dock'), value_type=bool)}]),
        Node(package='luna_robot', executable='face_bridge_node', name='luna_face_bridge', parameters=[params]),
        Node(package='luna_robot', executable='mock_mcu_node', name='luna_mock_mcu', parameters=[params],
             condition=IfCondition(PythonExpression(["'", mock, "' == 'true' and '", sim, "' != 'true'"]))),
        Node(package='luna_robot', executable='sim2d_node', name='luna_sim2d',
             parameters=[params, {'start_soc': ParameterValue(LaunchConfiguration('sim_soc'), value_type=float)}],
             condition=IfCondition(sim)),
        Node(package='micro_ros_agent', executable='micro_ros_agent', arguments=['serial', '--dev', LaunchConfiguration('mcu_port')],
             condition=IfCondition(PythonExpression(["'", mock, "' != 'true' and '", sim, "' != 'true'"]))),
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
