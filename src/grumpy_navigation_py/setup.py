from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'grumpy_navigation_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ost',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'planner = grumpy_navigation_py.planner_node:main',
            # 'controller = grumpy_navigation_py.controller_node:main',

            # 'planner = grumpy_navigation_py.simple_planner_node:main',
            # 'planner = grumpy_navigation_py.simple_planner_node_2:main',
            'planner = grumpy_navigation_py.planner_node_ms3:main',

            # 'controller = grumpy_navigation_py.path_controller_node:main',
            'controller = grumpy_navigation_py.controller_node_ms3:main',

            'fake_obstacles = grumpy_navigation_py.fake_obsctacle_node:main',

            'move_backwards= grumpy_navigation_py.move_backwards_node:main',
            'obstacle_detection= grumpy_navigation_py.obstacle_detection_node:main',
            'approach_goal= grumpy_navigation_py.approach_goal_node:main',


        ],
    },
)
