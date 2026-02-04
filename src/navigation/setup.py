from setuptools import setup, find_packages
import os

package_name = 'navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         ['launch/lidar_avoidance.launch.py']),
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
        ],
    },
)