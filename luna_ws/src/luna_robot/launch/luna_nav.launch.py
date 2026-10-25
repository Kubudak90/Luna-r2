"""Luna with mapping, navigation and go-to-dock.
  ros2 launch luna_robot luna_nav.launch.py                                   2D simulator, build a map
  ros2 launch luna_robot luna_nav.launch.py slam_mode:=localization          2D simulator, saved map
  ros2 launch luna_robot luna_nav.launch.py sim:=false mock:=false lidar:=true slam_mode:=localization
  ros2 launch luna_robot luna_nav.launch.py world:=table obstacle_sensors:=lidar   obstacle course, LiDAR only
obstacle_sensors:=all (default) | lidar: what Nav2 avoids with (launch/nav2.launch.py); the sensor nodes run either way.
world:= the simulator's room (core/sim_worlds.py); depth:= see luna.launch.py.
Go to the dock: ros2 topic pub --once /luna/nav/dock std_msgs/String "{data: go}"   (also starts on low battery)
The dock pose is kept in <map>.dock.yaml; mapping mode starts without one (except in the simulator, which knows
where its dock is) and learns it on the first docking that reaches CHARGING."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node

from luna_robot.core.sim_worlds import world


def dock_nav(context):
    cfg = context.launch_configurations
    share = get_package_share_directory('luna_robot')
    map_base = os.path.expanduser(cfg['map'])
    dock_file = map_base + '.dock.yaml'
    actions = []
    if cfg['slam_mode'] == 'mapping' and os.path.exists(dock_file):
        # A new map has a new map frame, so the old dock pose would send the robot to the wrong place. Without it
        # go-to-dock only falls back to dock_node's tag search until the first docking learns the new pose.
        os.replace(dock_file, dock_file + '.old')
        actions.append(LogInfo(msg=f'mapping: moved the old dock pose aside to {dock_file}.old'))
    params = [os.path.join(share, 'config', 'luna.yaml'),
              {'dock_file': dock_file, 'auto': cfg['auto_go_dock'] == 'true'}]
    if cfg['sim'] == 'true':
        params.append({'dock_pose': list(world(cfg['world']).dock)})   # sim world = map frame when it starts at 0,0,0
    return actions + [Node(package='luna_robot', executable='dock_nav_node', name='luna_dock_nav', parameters=params)]


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('luna_robot'), 'launch')
    include = lambda name, **args: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, name)), launch_arguments=args.items())
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('sim_soc', default_value='0.6'),
        DeclareLaunchArgument('slam_mode', default_value='mapping', description='mapping | localization'),
        DeclareLaunchArgument('map', default_value='~/.luna/maps/home', description='pose graph path, no extension'),
        DeclareLaunchArgument('auto_go_dock', default_value='true', description='go to the dock on low battery'),
        DeclareLaunchArgument('obstacle_sensors', default_value='all', description='all | lidar'),
        DeclareLaunchArgument('depth', default_value='', description='sim | gemini | off (luna.launch.py)'),
        DeclareLaunchArgument('world', default_value='room', description='simulator room (core/sim_worlds.py)'),
        # dock_nav_node decides when to dock, so dock_node must not start a tag search from wherever it is
        include('luna.launch.py', sim=C('sim'), mock=C('mock'), lidar=C('lidar'), sim_soc=C('sim_soc'),
                auto_dock='false', depth=C('depth'), sim_world=C('world')),
        include('slam.launch.py', slam_mode=C('slam_mode'), map=C('map')),
        include('nav2.launch.py', obstacle_sensors=C('obstacle_sensors')),
        OpaqueFunction(function=dock_nav),
    ])
