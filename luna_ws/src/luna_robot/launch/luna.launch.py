"""Luna bringup.  ros2 launch luna_robot luna.launch.py            (mock MCU, no hardware)
               ros2 launch luna_robot luna.launch.py sim:=true    (2D room simulator with LiDAR, for SLAM/Nav2)
               ros2 launch luna_robot luna.launch.py mock:=false  (Teensy over micro-ROS + RPLIDAR C1)
Mapping and navigation on top of this: luna_nav.launch.py.

tags:= where /luna/dock/tag comes from (default: mock with mock:=true, usb with mock:=false)
  mock  the mock MCU fakes it; no camera packages needed
  sim   sim_camera_node renders the head cameras from the mock's true pose → apriltag_ros → dock_tag_node
  usb   the two Arducam head cameras (usb_cam) → apriltag_ros → dock_tag_node
With sim:=true the 2D simulator replaces the mock MCU and publishes /luna/dock/tag itself, so tags:= must stay
empty or mock.
Voice: voice:=false leaves out audio/attention/assistant/speech; doa:=usb|mock|off (XVF3800 direction of arrival;
use doa:=mock or voice:=false with mock:=true / sim:=true); cloud:=true is the owner's consent for the cloud
assistant (it also needs a key, see config/luna.yaml)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from luna_robot.launch_util import IGNORE_LATE_SIGINT
CAMERAS = ('left', 'right')


def _tags(context):
    mock = context.perform_substitution(LaunchConfiguration('mock')).lower() in ('true', '1')
    sim = context.perform_substitution(LaunchConfiguration('sim')).lower() in ('true', '1')
    tags = context.perform_substitution(LaunchConfiguration('tags')) or ('mock' if mock or sim else 'usb')
    if tags not in ('mock', 'sim', 'usb'):
        raise ValueError(f'tags:={tags}: expected mock, sim or usb')
    if sim and tags != 'mock':
        raise ValueError(f'tags:={tags} with sim:=true: the 2D simulator publishes /luna/dock/tag itself')
    if tags in ('mock', 'sim') and not (mock or sim):
        raise ValueError(f'tags:={tags} needs mock:=true (it uses the mock robot pose)')
    return mock, sim, tags


def _tag_pipeline(context, params, share):
    mock, sim, tags = _tags(context)
    actions = []
    if sim:                      # sim2d_node (below) is the robot and fakes the tag
        return actions
    if mock:
        actions.append(Node(package='luna_robot', executable='mock_mcu_node', name='luna_mock_mcu',
                            parameters=[params, {'publish_tag': tags == 'mock'}]))
    if tags == 'mock':
        return actions
    # Camera images between processes: Fast DDS' default 512 KB shared-memory segments are too small for 720p, and
    # the UDP fallback stalls for up to ~2 s (config/fastdds_large_images.xml). Applies to the nodes started below;
    # a profile the user has set already wins. Needs /dev/shm ≥ 256 MB (docker/compose.yaml shm_size).
    if 'FASTRTPS_DEFAULT_PROFILES_FILE' not in os.environ:
        actions.insert(0, SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                                                 os.path.join(share, 'config', 'fastdds_large_images.xml')))
    if tags == 'sim':
        actions.append(Node(package='luna_robot', executable='sim_camera_node', name='luna_sim_camera', parameters=[params]))
    for cam in CAMERAS:
        ns = f'camera_{cam}'
        if tags == 'usb':
            actions.append(Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam', namespace=ns,
                                parameters=[os.path.join(share, 'config', 'cameras.yaml')],
                                prefix=IGNORE_LATE_SIGINT, exec_name=f'usb_cam_{cam}'))
        # apriltag_ros wants a rectified `image_rect`; until the cameras are calibrated the raw image is used.
        actions.append(Node(package='apriltag_ros', executable='apriltag_node', name='apriltag', namespace=ns,
                            parameters=[os.path.join(share, 'config', 'apriltag.yaml'), {'tag.frames': [f'dock_tag_{cam}']}],
                            remappings=[('image_rect', 'image_raw')], prefix=IGNORE_LATE_SIGINT, exec_name=f'apriltag_{cam}'))
    actions.append(Node(package='luna_robot', executable='dock_tag_node', name='luna_dock_tag', parameters=[params]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('luna_robot')
    params = os.path.join(share, 'config', 'luna.yaml')
    with open(os.path.join(share, 'urdf', 'luna.urdf')) as f:
        urdf = f.read()
    mock = LaunchConfiguration('mock')
    sim = LaunchConfiguration('sim')
    lidar = LaunchConfiguration('lidar')
    voice = IfCondition(LaunchConfiguration('voice'))
    actions = [
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('sim', default_value='false', description='2D simulator instead of the mock MCU'),
        DeclareLaunchArgument('sim_soc', default_value='0.6', description='simulator start state of charge 0..1'),
        DeclareLaunchArgument('auto_dock', default_value='true',
                              description='dock_node docks by itself on low battery (false when dock_nav_node decides)'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('tags', default_value='', description='mock | sim | usb (default: mock if mock:=true, else usb)'),
        OpaqueFunction(function=lambda context: _tags(context) and []),   # bad mock/tags: fail before any node starts
        DeclareLaunchArgument('voice', default_value='true'),
        DeclareLaunchArgument('doa', default_value='usb'),
        DeclareLaunchArgument('cloud', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': urdf}],
             prefix=IGNORE_LATE_SIGINT, exec_name='robot_state_publisher'),
        Node(package='luna_robot', executable='base_node', name='luna_base', parameters=[params]),
        Node(package='luna_robot', executable='head_node', name='luna_head', parameters=[params]),
        Node(package='luna_robot', executable='dock_node', name='luna_dock',
             parameters=[params, {'auto_dock': ParameterValue(LaunchConfiguration('auto_dock'), value_type=bool)}]),
        Node(package='luna_robot', executable='face_bridge_node', name='luna_face_bridge', parameters=[params]),
        OpaqueFunction(function=_tag_pipeline, args=[params, share]),   # mock MCU and/or the tag pipeline
        Node(package='luna_robot', executable='sim2d_node', name='luna_sim2d',
             parameters=[params, {'start_soc': ParameterValue(LaunchConfiguration('sim_soc'), value_type=float)}],
             condition=IfCondition(sim)),
        Node(package='micro_ros_agent', executable='micro_ros_agent', arguments=['serial', '--dev', LaunchConfiguration('mcu_port')],
             condition=IfCondition(PythonExpression(["'", mock, "' != 'true' and '", sim, "' != 'true'"]))),
        Node(package='luna_robot', executable='audio_node', name='luna_audio', condition=voice,
             parameters=[params, {'doa_source': ParameterValue(LaunchConfiguration('doa'), value_type=str)}]),
        Node(package='luna_robot', executable='attention_node', name='luna_attention', parameters=[params], condition=voice),
        Node(package='luna_robot', executable='assistant_node', name='luna_assistant', condition=voice,
             parameters=[params, {'cloud_enabled': ParameterValue(LaunchConfiguration('cloud'), value_type=bool)}]),
        Node(package='luna_robot', executable='speech_node', name='luna_speech', parameters=[params], condition=voice),
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
