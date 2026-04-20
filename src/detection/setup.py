from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'detection', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'detection', 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', 'detection', 'config'), glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dduberg',
    maintainer_email='danielduberg@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detection = detection.detection:main',
            'alter = detection.detection_alter:main',
            'alter_apr20 = detection.detection_alter_apr20:main',
        ],
    },
)
