from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # ← Config YAML — indispensable pour get_package_share_directory
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elyssa',
    maintainer_email='elyssa@robot.com',
    description='Bringup principal du robot — config YAML + launch global',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],  # pas de nodes ici, c'est un package de config/launch
    },
)