from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_planner_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ost',
    maintainer_email='magnus99ericson@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
#             'task_planner_node = task_planner_py.task_planner_node:main',
#             'task_planner_node_2 = task_planner_py.task_planner_node_2:main',
#             'task_planner_node_3 = task_planner_py.task_planner_node_3:main',
#             'task_planner_node_posearray = task_planner_py.task_planner_node_posearray:main',
#             'task_planner_node_posearray_2 = task_planner_py.task_planner_node_posearray_2:main',
#             'task_planner_node_posearray_3 = task_planner_py.task_planner_node_posearray_3:main',
#             'fake_arm_node = task_planner_py.fake_arm_node:main',
            'task_planner = task_planner_py.task_planner_ms3:main',

        ],
    },
)
