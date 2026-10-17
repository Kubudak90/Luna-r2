"""Static checks of the SLAM / Nav2 configuration: the robot's numbers, and the rule that Nav2 reaches the tracks
only through /cmd_vel, i.e. base_node's mux and safety gate. The YAML checks need PyYAML (present wherever ROS is;
skipped on a bare Mac Python). Run from luna_ws/src/luna_robot:  python3 -m unittest discover -s test"""
import ast
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from luna_robot import topics as T
from luna_robot.core import kinematics as K
from luna_robot.core.safety import SafetyConfig
from luna_robot.core.sim2d import HALF_LENGTH, HALF_WIDTH

try:
    import yaml
except ImportError:
    yaml = None

PKG = os.path.join(os.path.dirname(__file__), '..')
NAV_FILES = ('config/nav2.yaml', 'config/slam_mapping.yaml', 'config/slam_localization.yaml',
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
        self.assertIn('<link name="laser"/>', read('urdf/luna.urdf'))
        self.assertIn('xyz="-0.070 0 0.580"', read('urdf/luna.urdf'))

    def test_slam_matches_the_rplidar_c1(self):
        self.assertEqual(self.slam['scan_topic'], T.SCAN)
        self.assertEqual(self.slam['max_laser_range'], 12.0)
        self.assertEqual(self.slam['mode'], 'mapping')
        self.assertEqual(self.loc['mode'], 'localization')
        self.assertEqual(self.loc['map_start_pose'], [0.0, 0.0, 0.0])          # where mapping started


if __name__ == '__main__':
    unittest.main()
