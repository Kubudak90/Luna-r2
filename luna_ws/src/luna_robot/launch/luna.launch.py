"""Luna R2 bringup.  ros2 launch luna_robot luna.launch.py            (mock MCU, no hardware)
                  ros2 launch luna_robot luna.launch.py sim:=true    (2D room simulator with LiDAR, for SLAM/Nav2)
                  ros2 launch luna_robot luna.launch.py mock:=false  (Teensy over micro-ROS, RPLIDAR C1, Gemini 336)
Mapping and navigation on top of this: luna_nav.launch.py.

tags:= where /luna/dock/tag comes from (default: mock with mock:=true, gemini with mock:=false)
  mock    the mock MCU fakes it; no camera packages needed
  sim     sim_camera_node renders the Gemini colour camera from the mock's true pose → apriltag_ros → dock_tag_node
  gemini  the Gemini 336 colour camera (its driver comes with depth:=gemini) → apriltag_ros → dock_tag_node
With sim:=true the 2D simulator replaces the mock MCU and publishes /luna/dock/tag itself, so tags:= must stay
empty or mock.
Obstacles below the LiDAR plane: tof_range_node always turns the corner ToF into /luna/tof/* (Range) and point arcs.
depth:= where /camera/depth/points (and the gyro, and the colour camera) come from (default: sim with sim:=true,
gemini with mock:=false, else off)
  sim     sim_depth_node renders the 2.5D simulator's room (needs sim:=true); sim2d_node publishes the gyro
  gemini  Orbbec Gemini 336 via OrbbecSDK_ROS2 (optional like sllidar_ros2: skipped with a warning if absent;
          config/gemini.yaml, not tried on hardware)
  off     no depth camera
sim_world:= the simulator's room (core/sim_worlds.py: room, toy, table, chair, dock, door, door60/70/80).
face:= face_track_node on the Raspberry Pi Camera v2 over the screen (default: true with mock:=false): eye contact.
Voice: voice:=false leaves out audio/attention/stt/assistant/speech; doa:=usb|mock|off (ReSpeaker v2.0 direction of
arrival; use doa:=mock or voice:=false with mock:=true / sim:=true); cloud:=true is the owner's consent for the cloud
assistant (it also needs a key, see config/luna.yaml).
voice_servers:=true (default) also starts whisper.cpp's whisper-server (speech-to-text, whisper_model:=) and Piper's
HTTP server (Turkish voice, piper_voice:=) on the addresses luna_stt / luna_speech use in config/luna.yaml, each
only if its program and model are installed (otherwise a log line, and Luna says she cannot understand / stays
silent). Both run on the robot: no audio or text leaves it.
The face itself is a web page served by face_bridge_node (http://127.0.0.1:8765/) and shown full screen on the
head's AMOLED by a kiosk browser (tools/kiosk.sh, outside ROS)."""
import importlib.util
import os
import shutil
from urllib.parse import urlsplit

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from luna_robot.core.robot_model import drop_links
from luna_robot.launch_util import IGNORE_LATE_SIGINT, STOP_AS_CLEAN_EXIT
GEMINI_OPTICAL_FRAMES = ('camera_color_optical_frame', 'camera_depth_optical_frame')


def _truthy(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).lower() in ('true', '1')


def _tags(context):
    mock, sim = _truthy(context, 'mock'), _truthy(context, 'sim')
    tags = context.perform_substitution(LaunchConfiguration('tags')) or ('mock' if mock or sim else 'gemini')
    if tags not in ('mock', 'sim', 'gemini'):
        raise ValueError(f'tags:={tags}: expected mock, sim or gemini')
    if sim and tags != 'mock':
        raise ValueError(f'tags:={tags} with sim:=true: the 2D simulator publishes /luna/dock/tag itself')
    if tags in ('mock', 'sim') and not (mock or sim):
        raise ValueError(f'tags:={tags} needs mock:=true (it uses the mock robot pose)')
    if tags == 'gemini' and _depth_source(context) != 'gemini':
        raise ValueError('tags:=gemini needs depth:=gemini: the colour camera comes with the Gemini driver')
    return mock, sim, tags


def _depth_source(context):
    mock, sim = _truthy(context, 'mock'), _truthy(context, 'sim')
    depth = context.perform_substitution(LaunchConfiguration('depth')) or ('sim' if sim else 'off' if mock else 'gemini')
    if depth not in ('sim', 'gemini', 'off'):
        raise ValueError(f'depth:={depth}: expected sim, gemini or off')
    if depth == 'sim' and not sim:
        raise ValueError('depth:=sim needs sim:=true (it renders the simulator\'s room)')
    return depth


def _checks(context):
    _tags(context)          # bad mock/tags/depth: fail before any node starts
    return []


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
    # apriltag_ros wants a rectified `image_rect`; the Gemini colour stream is close enough to pinhole at 720p.
    actions.append(Node(package='apriltag_ros', executable='apriltag_node', name='apriltag', namespace='camera/color',
                        parameters=[os.path.join(share, 'config', 'apriltag.yaml'), {'tag.frames': ['dock_tag_color']}],
                        remappings=[('image_rect', 'image_raw')], prefix=IGNORE_LATE_SIGINT, exec_name='apriltag_color'))
    actions.append(Node(package='luna_robot', executable='dock_tag_node', name='luna_dock_tag', parameters=[params]))
    return actions


def _robot_state_publisher(context, urdf):
    if _depth_source(context) == 'gemini':     # OrbbecSDK_ROS2 publishes the optical frames below camera_link
        urdf = drop_links(urdf, GEMINI_OPTICAL_FRAMES)
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': urdf}], prefix=IGNORE_LATE_SIGINT, exec_name='robot_state_publisher')]


def _depth(context, params):
    depth = _depth_source(context)
    world = context.perform_substitution(LaunchConfiguration('sim_world'))
    if depth == 'sim':
        return [Node(package='luna_robot', executable='sim_depth_node', name='luna_sim_depth',
                     parameters=[params, {'world': world}])]
    if depth == 'off':
        return []
    try:  # OrbbecSDK_ROS2 is optional (built from source on the robot); the camera pose stays in our URDF
        driver = get_package_share_directory('orbbec_camera')
    except Exception:
        return [LogInfo(msg='depth:=gemini but orbbec_camera (OrbbecSDK_ROS2) is not installed: no depth camera')]
    with open(os.path.join(get_package_share_directory('luna_robot'), 'config', 'gemini.yaml')) as f:
        args = {k: str(v) for k, v in yaml.safe_load(f).items()}
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(driver, 'launch', 'gemini_330_series.launch.py')),
        launch_arguments=args.items())]


def _face(context, params):
    face = context.perform_substitution(LaunchConfiguration('face'))
    on = face.lower() in ('true', '1') if face else not (_truthy(context, 'mock') or _truthy(context, 'sim'))
    return [Node(package='luna_robot', executable='face_track_node', name='luna_face_track', parameters=[params])] if on else []


def _host_port(url, default_port):
    parts = urlsplit(url)
    return parts.hostname or '127.0.0.1', str(parts.port or default_port)


def _voice_servers(context, params):
    """whisper-server for luna_stt and Piper's HTTP server for luna_speech, bound where config/luna.yaml points them."""
    if not (_truthy(context, 'voice') and _truthy(context, 'voice_servers')):
        return []
    with open(params) as f:
        cfg = yaml.safe_load(f)
    stt = cfg.get('luna_stt', {}).get('ros__parameters', {})
    tts = cfg.get('luna_speech', {}).get('ros__parameters', {})
    actions = []
    if stt.get('stt_engine') == 'whisper_server':
        model = context.perform_substitution(LaunchConfiguration('whisper_model'))
        exe = shutil.which('whisper-server')
        if exe and os.path.isfile(model):
            host, port = _host_port(stt.get('stt_url', ''), 8178)
            actions.append(ExecuteProcess(
                cmd=[exe, '-m', model, '-l', stt.get('language', 'tr'), '--host', host, '--port', port, '-sns'],
                name='whisper_server', output='log', prefix=STOP_AS_CLEAN_EXIT))
        else:
            actions.append(LogInfo(msg='speech-to-text server not started: '
                                   + ('whisper-server not on PATH' if not exe else f'no model at {model}')))
    if tts.get('tts_engine') == 'piper_http':
        voice = context.perform_substitution(LaunchConfiguration('piper_voice'))
        if importlib.util.find_spec('piper') and os.path.isfile(voice):
            host, port = _host_port(tts.get('tts_url', ''), 5000)
            actions.append(ExecuteProcess(
                cmd=['python3', '-m', 'piper.http_server', '-m', voice, '--host', host, '--port', port],
                name='piper_server', output='log', prefix=STOP_AS_CLEAN_EXIT))
        else:
            actions.append(LogInfo(msg='TTS server not started: ' + ('piper-tts is not installed'
                                   if not importlib.util.find_spec('piper') else f'no voice at {voice}')))
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
        DeclareLaunchArgument('depth', default_value='',
                              description='sim | gemini | off (default: sim if sim:=true, gemini if mock:=false, else off)'),
        DeclareLaunchArgument('sim_world', default_value='room', description='simulator room (core/sim_worlds.py)'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('tags', default_value='',
                              description='mock | sim | gemini (default: mock if mock:=true, else gemini)'),
        DeclareLaunchArgument('face', default_value='', description='face camera tracking (default: true if mock:=false)'),
        OpaqueFunction(function=_checks),
        DeclareLaunchArgument('voice', default_value='true'),
        DeclareLaunchArgument('doa', default_value='usb'),
        DeclareLaunchArgument('cloud', default_value='false'),
        DeclareLaunchArgument('voice_servers', default_value='true',
                              description='start whisper-server and the Piper HTTP server when installed'),
        DeclareLaunchArgument('whisper_model', default_value='/models/whisper/ggml-large-v3-turbo-q5_0.bin'),
        DeclareLaunchArgument('piper_voice', default_value='/models/piper/tr_TR-dfki-medium.onnx'),
        OpaqueFunction(function=_robot_state_publisher, args=[urdf]),
        Node(package='luna_robot', executable='base_node', name='luna_base', parameters=[params]),
        Node(package='luna_robot', executable='dock_node', name='luna_dock',
             parameters=[params, {'auto_dock': ParameterValue(LaunchConfiguration('auto_dock'), value_type=bool)}]),
        Node(package='luna_robot', executable='face_bridge_node', name='luna_face_bridge', parameters=[params]),
        OpaqueFunction(function=_tag_pipeline, args=[params, share]),   # mock MCU and/or the tag pipeline
        Node(package='luna_robot', executable='sim2d_node', name='luna_sim2d',
             parameters=[params, {'start_soc': ParameterValue(LaunchConfiguration('sim_soc'), value_type=float),
                                  'world': LaunchConfiguration('sim_world')}],
             condition=IfCondition(sim)),
        Node(package='luna_robot', executable='tof_range_node', name='luna_tof_range', parameters=[params]),
        OpaqueFunction(function=_depth, args=[params]),
        OpaqueFunction(function=_face, args=[params]),
        Node(package='micro_ros_agent', executable='micro_ros_agent', arguments=['serial', '--dev', LaunchConfiguration('mcu_port')],
             condition=IfCondition(PythonExpression(["'", mock, "' != 'true' and '", sim, "' != 'true'"]))),
        Node(package='luna_robot', executable='audio_node', name='luna_audio', condition=voice,
             parameters=[params, {'doa_source': ParameterValue(LaunchConfiguration('doa'), value_type=str)}]),
        Node(package='luna_robot', executable='attention_node', name='luna_attention', parameters=[params], condition=voice),
        Node(package='luna_robot', executable='stt_node', name='luna_stt', parameters=[params], condition=voice),
        OpaqueFunction(function=_voice_servers, args=[params]),
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
