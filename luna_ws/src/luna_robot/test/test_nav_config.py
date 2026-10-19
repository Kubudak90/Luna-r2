"""Static checks of the SLAM / Nav2 configuration: the robot's numbers, and the rule that Nav2 reaches the tracks
only through /cmd_vel, i.e. base_node's mux and safety gate. The YAML checks need PyYAML (present wherever ROS is;
skipped on a bare Mac Python). Run from luna_ws/src/luna_robot:  python3 -m unittest discover -s test"""
import ast
import math
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot import topics as T
from luna_robot.core import kinematics as K
from luna_robot.core.safety import SafetyConfig
from luna_robot.core.sim2d import HALF_LENGTH, HALF_WIDTH
from luna_robot.core.tof import FRAMES, TofConfig

try:
    import yaml
except ImportError:
    yaml = None

PKG = os.path.join(os.path.dirname(__file__), '..')
NAV_FILES = ('config/nav2.yaml', 'config/nav2_obstacle_sensors.yaml', 'config/slam_mapping.yaml',
             'config/slam_localization.yaml',
             'launch/nav2.launch.py', 'launch/slam.launch.py', 'launch/luna_nav.launch.py')


def read(path):
    with open(os.path.join(PKG, path)) as f:
        return f.read()


def strings_in(path):
    """YAML without comments, or every string literal of a launch file except its docstring."""
    text = read(path)
    if path.endswith('.yaml'):
        return '\n'.join(line.split('#', 1)[0] for line in text.splitlines())
    tree = ast.parse(text)
    doc = ast.get_docstring(tree, clean=False)
    return '\n'.join(n.value for n in ast.walk(tree)
                     if isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value != doc)


def assigned(path, name):
    for node in ast.parse(read(path)).body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise KeyError(name)


class VelocityPath(unittest.TestCase):
    def test_nav_files_never_name_the_mcu_or_dock_velocity_topics(self):
        for path in NAV_FILES:
            text = strings_in(path)
            for topic in (T.MCU_WHEEL_TARGET, T.DOCK_CMD_VEL, 'wheel_target', 'dock/cmd_vel'):
                with self.subTest(path=path, topic=topic):
                    self.assertNotIn(topic, text)

    def test_collision_monitor_is_the_only_hop_onto_cmd_vel(self):
        text = strings_in('config/nav2.yaml')
        self.assertRegex(text, r'\n\s*cmd_vel_in_topic:\s*cmd_vel_smoothed\s*\n')
        self.assertRegex(text, r'\n\s*cmd_vel_out_topic:\s*' + re.escape(T.CMD_VEL) + r'\s*\n')
        self.assertEqual(len(re.findall(r'cmd_vel_out_topic', text)), 1)

    def test_every_nav2_velocity_goes_through_smoother_and_collision_monitor(self):
        servers, remaps = assigned('launch/nav2.launch.py', 'SERVERS'), assigned('launch/nav2.launch.py', 'REMAPS')
        self.assertIn('velocity_smoother', servers)
        self.assertIn('collision_monitor', servers)
        for name in ('controller_server', 'behavior_server', 'velocity_smoother'):
            self.assertIn(('cmd_vel', 'cmd_vel_nav'), remaps[name])
        targets = {dst for pairs in remaps.values() for _, dst in pairs}
        self.assertEqual(targets, {'cmd_vel_nav'})                    # nobody is remapped straight onto /cmd_vel

    def test_dock_node_auto_dock_is_off_under_dock_nav(self):
        # Otherwise dock_node starts its tag search (spinning in place) wherever the battery runs low.
        self.assertIn("auto_dock='false'", read('launch/luna_nav.launch.py'))


@unittest.skipIf(yaml is None, 'PyYAML not installed')
class Numbers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nav = yaml.safe_load(read('config/nav2.yaml'))
        cls.slam = yaml.safe_load(read('config/slam_mapping.yaml'))['slam_toolbox']['ros__parameters']
        cls.loc = yaml.safe_load(read('config/slam_localization.yaml'))['slam_toolbox']['ros__parameters']

    def p(self, *path):
        node = self.nav
        for key in path:
            node = node[key]
        return node

    def test_footprint_is_the_r06_track_base(self):
        want = [[HALF_LENGTH, HALF_WIDTH], [HALF_LENGTH, -HALF_WIDTH], [-HALF_LENGTH, -HALF_WIDTH],
                [-HALF_LENGTH, HALF_WIDTH]]
        for costmap in ('local_costmap', 'global_costmap'):
            with self.subTest(costmap=costmap):
                fp = yaml.safe_load(self.p(costmap, costmap, 'ros__parameters', 'footprint'))
                self.assertEqual(fp, want)
        self.assertAlmostEqual(2 * HALF_LENGTH, 0.340)
        self.assertAlmostEqual(2 * HALF_WIDTH, 0.378)

    def test_turning_in_place_stays_inside_the_planners_obstacle_margin(self):
        # NavFn (global costmap) treats cells within the inscribed radius (min half side + padding) as obstacles; RPP
        # turns Luna in place, sweeping the unpadded circumscribed circle: the first must cover the second (dock
        # course, shoe box). The controller's local costmap keeps the real footprint for its collision checks.
        sweep = math.hypot(HALF_LENGTH, HALF_WIDTH)
        glob = self.p('global_costmap', 'global_costmap', 'ros__parameters')
        pad = glob['footprint_padding']
        self.assertGreaterEqual(min(HALF_LENGTH, HALF_WIDTH) + pad, sweep)
        self.assertGreater(glob['inflation_layer']['inflation_radius'], math.hypot(HALF_LENGTH + pad, HALF_WIDTH + pad))
        self.assertLessEqual(2 * (HALF_LENGTH + pad), 0.68 - 0.1)       # the planner still fits beside the shoe box
        infl = glob['inflation_layer']
        door_cost = 252 * math.exp(-infl['cost_scaling_factor'] * (0.35 - (HALF_LENGTH + pad)))
        self.assertLess(door_cost, 253)                                 # a 0.70 m doorway stays plannable

    def test_recoveries_keep_what_only_the_camera_and_tof_saw(self):
        # ClearEntireCostmap wipes the voxel and ToF layers too: a low obstacle beside the robot is then forgotten.
        tree = read('behavior_trees/navigate_to_pose_keep_near_obstacles.xml')
        self.assertNotIn('<ClearEntireCostmap', tree)
        self.assertEqual(tree.count('<ClearCostmapExceptRegion'), 4)
        for costmap in ('local_costmap', 'global_costmap'):
            self.assertIn(f'service_name="{costmap}/clear_except_{costmap}"', tree)
        self.assertIn('navigate_to_pose_keep_near_obstacles.xml', read('launch/nav2.launch.py'))
        self.assertIn('nav2_clear_costmap_service_bt_node',
                      self.p('bt_navigator', 'ros__parameters', 'plugin_lib_names'))
        local = self.p('local_costmap', 'local_costmap', 'ros__parameters')
        self.assertLessEqual(local['footprint_padding'], 0.02)

    def test_speed_limits_stay_inside_the_base_limits(self):
        lim = K.DriveLimits()
        vs = self.p('velocity_smoother', 'ros__parameters')
        self.assertLessEqual(vs['max_velocity'][0], lim.max_linear)
        self.assertLessEqual(-vs['min_velocity'][0], lim.max_linear)
        self.assertLessEqual(vs['max_velocity'][2], lim.max_angular)
        self.assertLessEqual(-vs['min_velocity'][2], lim.max_angular)
        rpp = self.p('controller_server', 'ros__parameters', 'FollowPath')
        self.assertLessEqual(rpp['desired_linear_vel'], lim.max_linear)
        self.assertLessEqual(rpp['rotate_to_heading_angular_vel'], lim.max_angular)
        self.assertLessEqual(self.p('behavior_server', 'ros__parameters', 'max_rotational_vel'), lim.max_angular)

    def test_skid_steer_turns_in_place_and_slows_near_obstacles(self):
        rpp = self.p('controller_server', 'ros__parameters', 'FollowPath')
        self.assertTrue(rpp['use_rotate_to_heading'])
        self.assertFalse(rpp['allow_reversing'])
        self.assertTrue(rpp['use_cost_regulated_linear_velocity_scaling'])
        infl = self.p('local_costmap', 'local_costmap', 'ros__parameters', 'inflation_layer', 'cost_scaling_factor')
        self.assertEqual(rpp['inflation_cost_scaling_factor'], infl)
        cm = self.p('collision_monitor', 'ros__parameters')
        self.assertIn('slowdown', {cm[p]['action_type'] for p in cm['polygons']})
        self.assertNotIn('stop', {cm[p]['action_type'] for p in cm['polygons']})

    def test_controller_outpaces_the_gate_timeout(self):
        rate = self.p('controller_server', 'ros__parameters', 'controller_frequency')
        self.assertGreaterEqual(rate, 3 / SafetyConfig().cmd_timeout_s)          # ≥ 3 commands per timeout window
        self.assertGreaterEqual(self.p('velocity_smoother', 'ros__parameters', 'smoothing_frequency'), rate)

    def test_frames_match_base_node_and_urdf(self):
        for path in (('bt_navigator',), ('local_costmap', 'local_costmap'), ('global_costmap', 'global_costmap'),
                     ('behavior_server',)):
            with self.subTest(path=path):
                self.assertEqual(self.p(*path, 'ros__parameters')['robot_base_frame'], 'base_footprint')
        self.assertEqual(self.p('collision_monitor', 'ros__parameters', 'base_frame_id'), 'base_footprint')
        self.assertEqual((self.slam['base_frame'], self.slam['odom_frame'], self.slam['map_frame']),
                         ('base_footprint', 'odom', 'map'))
        urdf = read('urdf/luna.urdf')
        for frame in ('laser', 'oakd_link', 'oak_rgb_camera_optical_frame') + FRAMES:   # poses only there (R08)
            self.assertIn(f'<link name="{frame}"/>', urdf)

    def test_slam_matches_the_rplidar_c1(self):
        self.assertEqual(self.slam['scan_topic'], T.SCAN)
        self.assertEqual(self.slam['max_laser_range'], 12.0)
        self.assertEqual(self.slam['mode'], 'mapping')
        self.assertEqual(self.loc['mode'], 'localization')
        self.assertEqual(self.loc['map_start_pose'], [0.0, 0.0, 0.0])          # where mapping started


@unittest.skipIf(yaml is None, 'PyYAML not installed')
class ObstacleSensors(unittest.TestCase):
    """obstacle_sensors:=all overlays config/nav2_obstacle_sensors.yaml on nav2.yaml; lidar keeps nav2.yaml alone."""

    @classmethod
    def setUpClass(cls):
        cls.base = yaml.safe_load(read('config/nav2.yaml'))
        cls.over = yaml.safe_load(read('config/nav2_obstacle_sensors.yaml'))

    def costmaps(self, doc):
        return {name: doc[name][name]['ros__parameters'] for name in ('local_costmap', 'global_costmap')}

    def test_launch_overlays(self):
        overlays = assigned('launch/nav2.launch.py', 'OBSTACLE_SENSORS')
        self.assertEqual(overlays, {'lidar': [], 'all': ['nav2_obstacle_sensors.yaml']})
        self.assertIn("default_value='all'", read('launch/nav2.launch.py'))
        self.assertIn("obstacle_sensors=C('obstacle_sensors')", read('launch/luna_nav.launch.py'))

    def test_lidar_only_is_the_scan_alone(self):
        for name, cm in self.costmaps(self.base).items():
            with self.subTest(costmap=name):
                self.assertNotIn('voxel_layer', cm['plugins'])
                self.assertEqual(cm['obstacle_layer']['observation_sources'], 'scan')
        self.assertEqual(self.base['collision_monitor']['ros__parameters']['observation_sources'], ['scan'])

    def test_all_adds_camera_and_tof_layers_before_inflation(self):
        for name, cm in self.costmaps(self.over).items():
            with self.subTest(costmap=name):
                base = self.costmaps(self.base)[name]['plugins']
                self.assertEqual(cm['plugins'], base[:-1] + ['voxel_layer', 'tof_layer', 'inflation_layer'])
                vox, tof = cm['voxel_layer'], cm['tof_layer']
                self.assertEqual(vox['plugin'], 'nav2_costmap_2d::VoxelLayer')
                self.assertEqual(tof['plugin'], 'nav2_costmap_2d::ObstacleLayer')     # not RangeSensorLayer
                mark, clear = vox['depth_mark'], vox['depth_clear']
                self.assertEqual({mark['topic'], clear['topic']}, {T.DEPTH_POINTS})
                self.assertEqual((mark['marking'], mark['clearing'], clear['marking'], clear['clearing']),
                                 (True, False, False, True))
                self.assertAlmostEqual(mark['min_obstacle_height'], 0.03)
                self.assertAlmostEqual(mark['max_obstacle_height'], 0.62)
                self.assertLess(clear['min_obstacle_height'], 0.0)              # floor points clear
                self.assertGreaterEqual(vox['z_voxels'] * vox['z_resolution'], mark['max_obstacle_height'])
                self.assertLessEqual(vox['z_voxels'], 16)
                names = tof['observation_sources'].split()
                self.assertEqual(tuple(tof[n]['topic'] for n in names), T.TOF_POINTS)
                for n in names:
                    src = tof[n]
                    self.assertTrue(src['marking'] and src['clearing'])
                    # a max_range arc ("nothing nearer") clears but never marks
                    self.assertLess(src['obstacle_max_range'], TofConfig().max_m)
                    self.assertGreater(src['raytrace_max_range'], TofConfig().max_m)
                    self.assertLess(src['min_obstacle_height'], 0.101)              # the windows are at Z101

    def test_robot_top_fits_under_the_obstacle_band(self):
        from luna_robot.core.sim2d import robot_bands
        top = max(b.z1 for b in robot_bands())
        mark = self.costmaps(self.over)['local_costmap']['voxel_layer']['depth_mark']
        self.assertLess(top, mark['max_obstacle_height'])
        self.assertLess(mark['max_obstacle_height'] - top, 0.05)

    def test_collision_monitor_sources(self):
        cm = self.over['collision_monitor']['ros__parameters']
        self.assertEqual(cm['observation_sources'], ['scan', 'depth', 'tof_fl', 'tof_fr', 'tof_bl', 'tof_br'])
        self.assertEqual((cm['depth']['type'], cm['depth']['topic']), ('pointcloud', T.DEPTH_POINTS))
        self.assertAlmostEqual(cm['depth']['min_height'], 0.03)
        for name, topic in zip(cm['observation_sources'][2:], T.TOF_RANGES):
            self.assertEqual((cm[name]['type'], cm[name]['topic']), ('range', topic))
        base = self.base['collision_monitor']['ros__parameters']
        self.assertEqual(base['scan'], {'type': 'scan', 'topic': T.SCAN})   # the scan source itself is unchanged

    def test_tof_max_range_reaches_past_the_slowdown_box(self):
        # A "nothing nearer" reading puts collision_monitor's range points at max_range: they must stay outside
        # every polygon, or an empty cone would slow the robot down.
        cm = self.base['collision_monitor']['ros__parameters']
        slow_x = max(cm['PolygonSlow']['points'][0::2])
        self.assertGreater(HALF_LENGTH + TofConfig().max_m, slow_x + 0.2)


if __name__ == '__main__':
    unittest.main()
