from setuptools import find_packages, setup

package_name = 'odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'odometry = odometry.odometry:main',
            'odometry_dev_mag_3 = odometry.odometry_dev_mag_3:main',
            'odometry_only_encoders = odometry.odometry_only_encoders:main',
            'odometry_fixed = odometry.odometry_fixed:main',
            'odometry_fixed_no_imu = odometry.odometry_fixed_no_imu:main',
            'odometry_dev_mag_3_fixed = odometry.odometry_dev_mag_3_fixed:main',
            'ICP = odometry.ICP:main',
        ],
    },
)
