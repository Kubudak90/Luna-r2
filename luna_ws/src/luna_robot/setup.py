from glob import glob

from setuptools import find_packages, setup

package_name = 'luna_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + package_name + '/config/camera_info', glob('config/camera_info/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hüseyin Arslan',
    description='Luna companion robot ROS 2 nodes',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_node = luna_robot.base_node:main',
            'head_node = luna_robot.head_node:main',
            'dock_node = luna_robot.dock_node:main',
            'face_bridge_node = luna_robot.face_bridge_node:main',
            'mock_mcu_node = luna_robot.mock_mcu_node:main',
            'dock_tag_node = luna_robot.dock_tag_node:main',
            'sim_camera_node = luna_robot.sim_camera_node:main',
            'sim2d_node = luna_robot.sim2d_node:main',
            'dock_nav_node = luna_robot.dock_nav_node:main',
            'tof_range_node = luna_robot.tof_range_node:main',
            'slam_watchdog_node = luna_robot.slam_watchdog_node:main',
            'sim_depth_node = luna_robot.sim_depth_node:main',
            'audio_node = luna_robot.audio_node:main',
            'attention_node = luna_robot.attention_node:main',
            'assistant_node = luna_robot.assistant_node:main',
            'speech_node = luna_robot.speech_node:main',
        ],
    },
)
