from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # ← Important : juste le nom du package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elyssa',
    maintainer_email='elyssa@robot.com',
    description='Navigation package with obstacle avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = navigation.obstacle_avoidance_node:main',
            'lidar_test = navigation.lidar_test:main',
        ],
    },
)