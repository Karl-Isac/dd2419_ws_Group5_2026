from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learning_tf2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/learning_tf2_py/launch', ['launch/arm_init.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dduberg',
    maintainer_email='danielduberg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'executors = learning_tf2_py.executors:main',
            'interpolation = learning_tf2_py.interpolation:main',
            'threading = learning_tf2_py.threading:main',
            'timestamp = learning_tf2_py.timestamp:main',
            'pickup_hardcoded = learning_tf2_py.pickup_hardcoded:main',
            'pickup = learning_tf2_py.pickup:main',
            'random_nav = learning_tf2_py.random_nav:main',
            'arm_safe_republisher = learning_tf2_py.arm_safe_republisher:main',
            'jitter_test = learning_tf2_py.jitter_test:main',
            'arm_control = learning_tf2_py.arm_control:main',
            'arm_cube_detection_test = learning_tf2_py.arm_cube_detection_test:main',
            'exploration_mapper = learning_tf2_py.exploration_mapper:main',
            'exploration_mapper_new = learning_tf2_py.exploration_mapper_new:main',
            'keyboard = learning_tf2_py.keyboard_node:main',
        ],
    },
)
